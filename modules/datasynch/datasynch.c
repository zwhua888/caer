/*
 * datasynch.c
 *
 *  Created on: Oct 2016
 *      Author: federico.corradi@inilabs.com
 */

#include "datasynch.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"

struct DSFilter_state {
	// accessible from module settings
	int32_t deltaT;
	// privates
	int32_t minTs_cam0;
	int32_t maxTs_cam0;
	int32_t minTs_cam1;
	int32_t maxTs_cam1;
};

typedef struct DSFilter_state *DSFilterState;

static bool caerDataSynchFilterInit(caerModuleData moduleData);
static void caerDataSynchFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerDataSynchFilterConfig(caerModuleData moduleData);
static void caerDataSynchFilterExit(caerModuleData moduleData);
static void caerDataSynchFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);

static struct caer_module_functions caerDataSynchFilterFunctions = { .moduleInit =
	&caerDataSynchFilterInit, .moduleRun = &caerDataSynchFilterRun, .moduleConfig =
	&caerDataSynchFilterConfig, .moduleExit = &caerDataSynchFilterExit, .moduleReset =
	&caerDataSynchFilterReset };

void caerDataSynchFilter(uint16_t moduleID, caerPolarityEventPacket polarity_cam0,
		caerPolarityEventPacket polarity_cam1,
		caerFrameEventPacket frame_cam0, caerFrameEventPacket frame_cam1,
		caerIMU6EventPacket imu_cam0, caerIMU6EventPacket imu_cam1,
		caerSpecialEventPacket special_cam0, caerSpecialEventPacket special_cam1) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DSFilter", PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerDataSynchFilterFunctions, moduleData, sizeof(struct DSFilter_state), 8,
			polarity_cam0, polarity_cam1, frame_cam0, frame_cam1, imu_cam0, imu_cam1,
			special_cam0, special_cam1);
}

static bool caerDataSynchFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "deltaT", 30000);
	DSFilterState state = moduleData->moduleState;
	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	state->minTs_cam0 = 0;
	state->maxTs_cam0 = -1;
	state->minTs_cam1 = 0;
	state->maxTs_cam1 = -1;

	// Nothing that can fail here.
	return (true);
}

static void caerDataSynchFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity_cam0 = va_arg(args, caerPolarityEventPacket);
	caerPolarityEventPacket polarity_cam1 = va_arg(args, caerPolarityEventPacket);
	caerFrameEventPacket frame_cam0 = va_arg(args, caerFrameEventPacket);
	caerFrameEventPacket frame_cam1 = va_arg(args, caerFrameEventPacket);
	caerIMU6EventPacket imu_cam0 = va_arg(args, caerIMU6EventPacket);
	caerIMU6EventPacket imu_cam1 = va_arg(args, caerIMU6EventPacket);
	caerSpecialEventPacket special_cam0 = va_arg(args, caerSpecialEventPacket);
	caerSpecialEventPacket special_cam1 = va_arg(args, caerSpecialEventPacket);


	DSFilterState state = moduleData->moduleState;


	// Iterate over events streams and get packet time

	if(polarity_cam0 == NULL || polarity_cam1 == NULL){
		return;
	}

	//caerPolarityEventGet

	CAER_POLARITY_ITERATOR_VALID_START(polarity_cam0)
		// Get values on which to operate.
		int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity_cam0);
		if( ts < state->minTs_cam0){
			state->minTs_cam0 = ts;
		}
		if( ts > state->maxTs_cam0){
			state->maxTs_cam0 = ts;
		}
	CAER_POLARITY_ITERATOR_VALID_END
	CAER_POLARITY_ITERATOR_VALID_START(polarity_cam1)
		// Get values on which to operate.
		int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity_cam1);
		if( ts < state->minTs_cam1){
			state->minTs_cam1 = ts;
		}
		if( ts > state->maxTs_cam1){
			state->maxTs_cam1 = ts;
		}
	CAER_POLARITY_ITERATOR_VALID_END



}

static void caerDataSynchFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	DSFilterState state = moduleData->moduleState;

	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
}

static void caerDataSynchFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	DSFilterState state = moduleData->moduleState;

}

static void caerDataSynchFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	DSFilterState state = moduleData->moduleState;

}
