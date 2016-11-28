/*
 * backgroundactivityfilter.c
 *
 *  Created on: Jan 20, 2014
 *      Author: chtekk
 */

#include "arduinocnt.h"
#include "base/mainloop.h"
#include "base/module.h"
//#include "ext/ringbuffer/ringbuffer.h"

#include "arduino-serial-lib.h"

//majory voting thread
#ifdef HAVE_PTHREADS
#include "ext/c11threads_posix.h"
#endif
#include "stdatomic.h"

#define ROCK 3
#define PAPER 1
#define SCISSORS 2
#define BACKGROUND 4	//network output unit number one based (starting from one)
#define AVERAGEOVER 3

struct ASFilter_state {
	int fd;
	int baudRate;
	int timeout;
	char * serialPort;
	thrd_t majorityThread;
	atomic_bool running;
	uint16_t pos;
	uint16_t lastcommand;
	atomic_int_fast32_t decision[AVERAGEOVER];
//RingBuffer dataTransfer;
};

typedef struct ASFilter_state *ASFilterState;

static bool caerArduinoCNTInit(caerModuleData moduleData);
static void caerArduinoCNTRun(caerModuleData moduleData, size_t argsNumber,
		va_list args);
static void caerArduinoCNTConfig(caerModuleData moduleData);
static void caerArduinoCNTExit(caerModuleData moduleData);
static void caerArduinoCNTReset(caerModuleData moduleData,
		uint16_t resetCallSourceID);

int majorityThread(void *ASFilter_state);

static struct caer_module_functions caerArduinoCNTFunctions = { .moduleInit =
		&caerArduinoCNTInit, .moduleRun = &caerArduinoCNTRun, .moduleConfig =
		&caerArduinoCNTConfig, .moduleExit = &caerArduinoCNTExit, .moduleReset =
		&caerArduinoCNTReset };

void caerArduinoCNT(uint16_t moduleID, int result, bool *haveimage) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ArduinoCNT",
			PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerArduinoCNTFunctions, moduleData,
			sizeof(struct ASFilter_state), 2, result, haveimage);
}

int majorityThread(void *ASFilter_state) {

	if (ASFilter_state == NULL) {
		return (thrd_error);
	}

	ASFilterState state = ASFilter_state;

	thrd_set_name("ArduinoCNTThread");

	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		/*do majority voting*/

		for (size_t i = 0; i < AVERAGEOVER; i++) {
			atomic_load(&state->decision[i]);
		}
		int paper = 0;
		int rock = 0;
		int scissors = 0;
		int back =0;
		for (size_t i = 0; i < AVERAGEOVER; i++) {
			if (state->decision[i] == PAPER) {
				paper++;
			} else if (state->decision[i] == SCISSORS) {
				scissors++;
			} else if (state->decision[i] == ROCK) {
				rock++;
			} else if (state->decision[i] == BACKGROUND) {
				back++;
			}
		}

		char res[20];
		int16_t current_dec;
		if ((rock > paper) && (rock > scissors) && (rock > back)) {
			/*play rock*/
			sprintf(res, "%d", ROCK);
			current_dec = ROCK;
		} else if ((back > paper) && (back > scissors) && (back > rock)) {
			/*play back*/
			sprintf(res, "%d", BACKGROUND);
			current_dec = BACKGROUND;
		} else if ((scissors > paper) && (scissors > rock)
				&& (scissors > back)) {
			/*play scissors*/
			sprintf(res, "%d", SCISSORS);
			current_dec = SCISSORS;
		} else if ((paper > scissors) && (paper > rock) && (paper > back)) {
			/*play paper*/
			sprintf(res, "%d", PAPER);
			current_dec = PAPER;
		}else{
			current_dec = state->lastcommand;
		}
		if(current_dec != state->lastcommand){

			printf("\n\n\\n####################### sending %d\n\n", current_dec);
			serialport_write(state->fd, res);
			state->lastcommand = current_dec;
		}
		//printf("\n####################### sending %d\n", current_dec);


	}

	return (thrd_success);
}

static bool caerArduinoCNTInit(caerModuleData moduleData) {

	sshsNodePutStringIfAbsent(moduleData->moduleNode, "serialPort",
			"/dev/ttyUSB0");
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "baudRate", 115200);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "timeout", 5000);

	ASFilterState state = moduleData->moduleState;

	state->serialPort = (char*) calloc(1024, sizeof(char));
	state->serialPort = sshsNodeGetString(moduleData->moduleNode, "serialPort");
	state->baudRate = sshsNodeGetInt(moduleData->moduleNode, "baudRate");

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData,
			&caerModuleConfigDefaultListener);

	//open Serial PORT
	state->fd = -1;
	state->fd = serialport_init(state->serialPort, state->baudRate);
	if (state->fd == -1) {
		caerLog(CAER_LOG_CRITICAL, "arduinoCNT", "failed to open usb port");
		exit(1);
	}
	serialport_flush(state->fd);

	/*state->dataTransfer = ringBufferInit(AVERAGEOVER);
	 if (state->dataTransfer == NULL) {
	 caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
	 "ringbuffer failed to initialize");
	 return (NULL);
	 }*/

	atomic_store(&state->pos, 0);
	for (size_t i = 0; i < AVERAGEOVER; i++) {
		atomic_store(&state->decision[i], BACKGROUND);
	}
	state->lastcommand = BACKGROUND;
	state->pos = 0;

	//start thread for arm control
	if (thrd_create(&state->majorityThread, &majorityThread, state)
			!= thrd_success) {
		//ringBufferFree(state->dataTransfer);
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
				"Majority voting thread failed to initialize");
		exit (false);
	}else{
		atomic_store(&state->running,true);
	}


	// Nothing that can fail here.
	return (true);
}

static void caerArduinoCNTRun(caerModuleData moduleData, size_t argsNumber,
		va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	int * result = va_arg(args, int*);
	bool *haveimage = va_arg(args, bool*);

	ASFilterState state = moduleData->moduleState;

	if (haveimage[0]) {

		atomic_store(&state->decision[state->pos], result[0]);
		if (state->pos == AVERAGEOVER) {
			state->pos = 0;
		} else {
			state->pos = state->pos + 1;
		}

	}

}

static void caerArduinoCNTConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	ASFilterState state = moduleData->moduleState;

	state->serialPort = sshsNodeGetString(moduleData->moduleNode, "serialPort");
	state->baudRate = sshsNodeGetInt(moduleData->moduleNode, "baudRate");

}

static void caerArduinoCNTExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData,
			&caerModuleConfigDefaultListener);

	ASFilterState state = moduleData->moduleState;

	free(state->serialPort);

	//close tread
	atomic_store(&state->running, false);

	if ((errno = thrd_join(state->majorityThread, NULL)) != thrd_success) {
		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
				"failed to join majority voting thread error: %d\n", errno);
	}
	serialport_write(state->fd, "5\n");
	serialport_close(state->fd);
}

static void caerArduinoCNTReset(caerModuleData moduleData,
		uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	ASFilterState state = moduleData->moduleState;

}

