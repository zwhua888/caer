/*
 * backgroundactivityfilter.c
 *
 *  Created on: Jan 20, 2014
 *      Author: chtekk
 */

#include "arduinocnt.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/ringbuffer/ringbuffer.h"

#include "arduino-serial-lib.h"

//majory voting thread
#ifdef HAVE_PTHREADS
#include "ext/c11threads_posix.h"
#endif
#include "stdatomic.h"

#define ROCK 3
#define PAPER 1
#define SCISSORS 2
#define BACKGROUND 4
#define AVERAGEOVER 5

struct ASFilter_state {
	int fd;
	int baudRate;
	int timeout;
	char * serialPort;
	thrd_t majorityThread;
	atomic_bool running;
	RingBuffer dataTransfer;
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

void caerArduinoCNT(uint16_t moduleID, int result) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ArduinoCNT",
			PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerArduinoCNTFunctions, moduleData,
			sizeof(struct ASFilter_state), 1, result);
}

int majorityThread(void *ASFilter_state) {
	if (ASFilter_state == NULL) {
		return (thrd_error);
	}

	ASFilterState state = ASFilter_state;

	int decisions[AVERAGEOVER];
	for(size_t i=0; i < 5; i++){
		decisions[i] = BACKGROUND;
	}


	thrd_set_name("ArduinoCNTThread");

	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		/*do majority voting*/

		int tmp = ringBufferGet(state->dataTransfer);
		if (tmp == 0) {
			;
		} else {
			for (size_t i = 0; i < AVERAGEOVER - 1; i++) {
				decisions[i] = decisions[i + 1];
			}
			decisions[0] = tmp;
			int paper, rock, scissors, back;
			for (size_t i = 0; i < AVERAGEOVER; i++) {
				if (decisions[i] == PAPER) {
					paper++;
				} else if (decisions[i] == SCISSORS) {
					scissors++;
				} else if (decisions[i] == ROCK) {
					rock++;
				} else if (decisions[i] == BACKGROUND) {
					back++;
				}
			}
			char res;
			if( (rock > paper) && (rock > scissors) && (rock > back) ){
				/*play rock*/
				sprintf(res, "%d", ROCK);
				serialport_write(state->fd, res);
			}else if( (back > paper) && (back > scissors) && (back > rock) ){
				/*play back*/
				sprintf(res, "%d", BACKGROUND);
				serialport_write(state->fd, res);
			}else if( (scissors > paper) && (scissors > rock) && (scissors > back) ){
				/*play scissors*/
				sprintf(res, "%d", SCISSORS);
				serialport_write(state->fd, res);
			}else if( (paper > scissors) && (paper > rock) && (paper > back) ){
				/*play paper*/
				sprintf(res, "%d", PAPER);
				serialport_write(state->fd, res);
			}
		}
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

	state->dataTransfer = ringBufferInit(AVERAGEOVER);
	if (state->dataTransfer == NULL) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
				"ringbuffer failed to initialize");
		return (NULL);
	}

	//start thread for arm control
	if (thrd_create(&state->majorityThread, &majorityThread, state)
			!= thrd_success) {
		ringBufferFree(state->dataTransfer);
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
				"Majority voting thread failed to initialize");
		return (NULL);
	}

	// Nothing that can fail here.
	return (true);
}

static void caerArduinoCNTRun(caerModuleData moduleData, size_t argsNumber,
		va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	int * result = va_arg(args, int*);

	ASFilterState state = moduleData->moduleState;

	if (result[0] != NULL) {
		if (!ringBufferPut(state->dataTransfer, result[0])) {
			caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString,
					"Dropped decision because ringbuffer full");
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

