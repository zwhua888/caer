/* NullHop Zynq Interface cAER module
 *  Author: federico.corradi@inilabs.com
 */

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
static void caerNullHopWrapperRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerNullHopWrapperExit(caerModuleData moduleData);

static struct caer_module_functions caerNullHopWrapperFunctions = { .moduleInit = &caerNullHopWrapperInit, .moduleRun =
	&caerNullHopWrapperRun, .moduleConfig =
NULL, .moduleExit = &caerNullHopWrapperExit };

const char * caerNullHopWrapper(uint16_t moduleID, char ** file_string, double *classificationResults, int max_img_qty) {

    caerModuleData moduleData = caerMainloopFindModule(moduleID, "caerNullHopWrapper", PROCESSOR);
	caerModuleSM(&caerNullHopWrapperFunctions, moduleData, sizeof(struct nullhopwrapper_state), 3, file_string, classificationResults, max_img_qty);

	return (NULL);
}

static bool caerNullHopWrapperInit(caerModuleData moduleData) {

    nullhopwrapperState state = moduleData->moduleState;
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "detThreshold", 0.5);
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode, "detThreshold");

	//Initializing nullhop network..
	state->cpp_class = newzs_driverMonitor();
	zs_driverMonitor_initNet(state->cpp_class);

	return (true);
}

static void caerNullHopWrapperExit(caerModuleData moduleData) {
	nullhopwrapperState state = moduleData->moduleState;
	deleteMyClass(state->cpp_class); //free memory block
}

static void caerNullHopWrapperRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
    UNUSED_ARGUMENT(argsNumber);
    nullhopwrapperState state = moduleData->moduleState;
    char ** file_string = va_arg(args, char **);
    double *classificationResults = va_arg(args, double*);
    int max_img_qty = va_arg(args, int);

    //update module state
    state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode, "detThreshold");

    caerLog(CAER_LOG_ERROR, __func__, "inside NullHopLoop");

    for (int i = 0; i < max_img_qty; ++i){
        if (file_string[i] != NULL) {
        	zs_driverMonitor_initNet(state->cpp_class);
        	zs_driverMonitor_resetAxiBus(state->cpp_class);
        	zs_driverMonitor_file_set(state->cpp_class, file_string[i], &classificationResults[i], state->detThreshold);
        	//pid_t pid = fork();
        	//if(pid==0){/*child process*/
        	//	static char *argv[]={"echo","NullHop testbench", NULL};
        	//system("/home/root/zs_driver");
        	//	exit(127);/*only if execv fails */
        	//}else{
        	//	waitpid(pid,0,0);
        	//}
        }
    }

    return;
}
