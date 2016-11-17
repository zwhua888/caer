/* NullHop Zynq Interface cAER module
 *  Author: federico.corradi@inilabs.com
 */
#include "main.h"
#include <libcaer/events/frame.h>

#include "nullhopinterface.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "wrapper.h"
#include <sys/types.h>
#include <sys/wait.h>

struct nullhopwrapper_state {
	uint32_t *integertest;
	char * file_to_classify;
	double detThreshold;
	struct MyClass* cpp_class; //pointer to cpp_class_object
};

typedef struct nullhopwrapper_state *nullhopwrapperState;

static bool caerNullHopWrapperInit(caerModuleData moduleData);
static void caerNullHopWrapperRun(caerModuleData moduleData, size_t argsNumber,
		va_list args);
static void caerNullHopWrapperExit(caerModuleData moduleData);

static struct caer_module_functions caerNullHopWrapperFunctions = {
		.moduleInit = &caerNullHopWrapperInit, .moduleRun =
				&caerNullHopWrapperRun, .moduleConfig =
		NULL, .moduleExit = &caerNullHopWrapperExit };

const char * caerNullHopWrapper(uint16_t moduleID,
		caerFrameEventPacket imagestreamer) {

	caerModuleData moduleData = caerMainloopFindModule(moduleID,
			"caerNullHopWrapper", PROCESSOR);
	caerModuleSM(&caerNullHopWrapperFunctions, moduleData,
			sizeof(struct nullhopwrapper_state), 1, imagestreamer);

	return (NULL);
}

static bool caerNullHopWrapperInit(caerModuleData moduleData) {

	nullhopwrapperState state = moduleData->moduleState;
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "detThreshold", 0.5);
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode,
			"detThreshold");

	//Initializing nullhop network..
	state->cpp_class = newzs_driverMonitor();

	zs_driverMonitor_initNet(state->cpp_class);
	zs_driverMonitor_launchThread(state->cpp_class);
	//zs_driverMonitor_resetAxiBus(state->cpp_class);

	return (true);
}

static void caerNullHopWrapperExit(caerModuleData moduleData) {
	nullhopwrapperState state = moduleData->moduleState;

	//zs_driverMonitor_closeThread(state->cpp_class); // join
	//deleteMyClass(state->cpp_class); //free memory block
}

static void caerNullHopWrapperRun(caerModuleData moduleData, size_t argsNumber,
		va_list args) {
	UNUSED_ARGUMENT(argsNumber);
	caerFrameEventPacket imagestreamer_hists = va_arg(args,
			caerFrameEventPacket*);

	if (imagestreamer_hists == NULL) {
		return;
	}

	nullhopwrapperState state = moduleData->moduleState;

	//update module state
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode,
			"detThreshold");

	//zs_driverMonitor_loadImage(state->cpp_class);
	CAER_FRAME_ITERATOR_ALL_START(imagestreamer_hists)


		zs_driverMonitor_threadExists(state->cpp_class);

		uint16_t *picture = (uint16_t *) caerFrameEventGetPixelArrayUnsafe(caerFrameIteratorElement);

		zs_driverMonitor_file_set(state->cpp_class, picture);

	CAER_FRAME_ITERATOR_ALL_END
	return;
}
