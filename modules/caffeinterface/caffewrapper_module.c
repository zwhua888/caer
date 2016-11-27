/* Caffe Interface cAER module
 *  Author: federico.corradi@inilabs.com
 */

#include "base/mainloop.h"
#include "base/module.h"
#include "wrapper.h"

struct caffewrapper_state {
	uint32_t *integertest;
	char * file_to_classify;
	double detThreshold;
	bool doPrintOutputs;
	bool doShowActivations;
	struct MyClass* cpp_class; //pointer to cpp_class_object
};

typedef struct caffewrapper_state *caffewrapperState;

static bool caerCaffeWrapperInit(caerModuleData moduleData);
static void caerCaffeWrapperRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerCaffeWrapperExit(caerModuleData moduleData);

static struct caer_module_functions caerCaffeWrapperFunctions = { .moduleInit = &caerCaffeWrapperInit, .moduleRun =
	&caerCaffeWrapperRun, .moduleConfig =
NULL, .moduleExit = &caerCaffeWrapperExit };

const char * caerCaffeWrapper(uint16_t moduleID,
		int * imagestreamer, bool * haveimg, int* result, int size) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "caerCaffeWrapper", PROCESSOR);
	if (moduleData == NULL) {
		return (NULL);
	}

	caerModuleSM(&caerCaffeWrapperFunctions, moduleData, sizeof(struct caffewrapper_state), 4, imagestreamer, haveimg, result, size);

	return (NULL);
}

static bool caerCaffeWrapperInit(caerModuleData moduleData) {

	caffewrapperState state = moduleData->moduleState;
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "detThreshold", 0.96);
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode, "detThreshold");
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doPrintOutputs", false);
	state->doPrintOutputs = sshsNodeGetBool(moduleData->moduleNode, "doPrintOutputs");
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doShowActivations", false);
	state->doShowActivations = sshsNodeGetBool(moduleData->moduleNode, "doShowActivations");

	//Initializing caffe network..
	state->cpp_class = newMyClass();
	MyClass_init_network(state->cpp_class);

	return (true);
}

static void caerCaffeWrapperExit(caerModuleData moduleData) {
	caffewrapperState state = moduleData->moduleState;
	deleteMyClass(state->cpp_class); //free memory block
}

static void caerCaffeWrapperRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	int * imagestreamer_hists = va_arg(args, int*);
	bool * haveimg = va_arg(args, bool*);
	int * result = va_arg(args, int*);

	if (imagestreamer_hists == NULL) {
		return;
	}

	caffewrapperState state = moduleData->moduleState;

	/*for (int i = 0; i < max_img_qty; ++i) {
		if (file_string[i] != NULL) {
			MyClass_file_set(state->cpp_class, file_string[i], &classificationResults[i], state->detThreshold,
				state->doPrintOutputs, single_frame, state->doShowActivations);
		}
	}*/
	if(haveimg[0] == true){
		result[0] = MyClass_file_set(state->cpp_class, imagestreamer_hists, size);
	}

	//update module state
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode, "detThreshold");
	state->doPrintOutputs = sshsNodeGetBool(moduleData->moduleNode, "doPrintOutputs");
	state->doShowActivations = sshsNodeGetBool(moduleData->moduleNode, "doShowActivations");


	return;
}
