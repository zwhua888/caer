#include "cameracalibration.h"
#include "calibration_settings.h"
#include "calibration_wrapper.h"
#include "base/mainloop.h"
#include "base/module.h"


struct MultiCalibrationState_struct {
	struct MultiCalibrationSettings_struct settings; // Struct containing all settings (shared)
	struct MultiCalibrarion *cpp_class; // Pointer to cpp_class_object
	uint64_t lastFrameTimestamp;
	size_t lastFoundPoints;
	bool calibrationLoaded;
};

typedef struct MultiCalibrationState_struct *MultiCalibrationState;

static bool caerMultiCalibrationInit(caerModuleData moduleData);
static void caerMultiCalibrationRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerMultiCalibrationConfig(caerModuleData moduleData);
static void caerMultiCalibrationExit(caerModuleData moduleData);
static void updateSettings(caerModuleData moduleData);

static struct caer_module_functions caerMultiCalibrationFunctions = { .moduleInit = &caerMultiCalibrationInit,
	.moduleRun = &caerMultiCalibrationRun, .moduleConfig = &caerMultiCalibrationConfig, .moduleExit =
		&caerMultiCalibrationExit };

void caerMultiCalibration(uint16_t moduleID, caerFrameEventPacket frame_0, caerFrameEventPacket frame_1 ) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "LaserMultiEstimation", PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerMultiCalibrationFunctions, moduleData, sizeof(struct MultiCalibrationState_struct), 2, frame_0,
		frame_1);
}

static bool caerMultiCalibrationInit(caerModuleData moduleData) {
	MultiCalibrationState state = moduleData->moduleState;

	// Create config settings.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doCalibration", false); // Do calibration using live images
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doSavetxt", false);
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "saveFileName", "multi_camera_results.txt");
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "loadFileName_cam0", "camera_calib_1.xml"); // The name of the file from which to load the calibration
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "loadFileName_cam1", "camera_calib_0.xml"); // The name of the file from which to load the calibration
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "captureDelay", 500000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "nCamera", 2);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "patternWidth", 800);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "patternHeight", 600);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "showFeatureExtraction", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "verbose", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "nMiniMatches", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "cameraType", 0);

	// Update all settings.
	updateSettings(moduleData);

	// Initialize C++ class for OpenCV integration.
	state->cpp_class = multicalibration_init(&state->settings);
	if (state->cpp_class == NULL) {
		return (false);
	}

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);
	//not loaded at the init
         state->calibrationLoaded = false;

	return (true);
}

static void updateSettings(caerModuleData moduleData) {
    MultiCalibrationState state = moduleData->moduleState;

    state->settings.doCalibration = sshsNodeGetBool(moduleData->moduleNode, "doCalibration");
    state->settings.doSavetxt = sshsNodeGetBool(moduleData->moduleNode, "doSavetxt");
    state->settings.saveFileName = sshsNodeGetString(moduleData->moduleNode, "saveFileName");
    state->settings.loadFileNames = sshsNodeGetString(moduleData->moduleNode, "loadFileNames");
    state->settings.nCamera = sshsNodeGetInt(moduleData->moduleNode, "nCamera");
    state->settings.patternWidth = sshsNodeGetInt(moduleData->moduleNode, "patternWidth");
    state->settings.patternHeight = sshsNodeGetInt(moduleData->moduleNode, "patternHeight");
    state->settings.showFeatureExtraction = sshsNodeGetInt(moduleData->moduleNode, "showFeatureExtraction");
    state->settings.verbose = sshsNodeGetInt(moduleData->moduleNode, "verbose");
    state->settings.nMiniMatches = sshsNodeGetInt(moduleData->moduleNode, "nMiniMatches");
    state->settings.cameraType = sshsNodeGetInt(moduleData->moduleNode, "cameraType");


}

static void caerMultiCalibrationConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	MultiCalibrationState state = moduleData->moduleState;

}

static void caerMultiCalibrationExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	MultiCalibrationState state = moduleData->moduleState;

	//Multicalibration_destroy(state->cpp_class);

	free(state->settings.saveFileName);
	free(state->settings.loadFileNames);

}

static void caerMultiCalibrationRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerFrameEventPacket frame_0 = va_arg(args, caerFrameEventPacket);
	caerFrameEventPacket frame_1 = va_arg(args, caerFrameEventPacket);

	MultiCalibrationState state = moduleData->moduleState;

    // At this point we always try to load the calibration settings for undistortion.
	// Maybe they just got created or exist from a previous run.
	if (!state->calibrationLoaded) {
		state->calibrationLoaded = multicalibration_loadCalibrationFile(state->cpp_class, &state->settings);
	}

        // Marker Multi estimation is done only using frames.
	if (state->settings.doCalibration && frame_0 != NULL && frame_1 != NULL) {
		CAER_FRAME_ITERATOR_VALID_START(frame_0)
			// Only work on new frames if enough time has passed between this and the last used one.
			uint64_t currTimestamp_0 = U64T(caerFrameEventGetTSStartOfFrame64(caerFrameIteratorElement, frame_0));
			// If enough time has passed, try to add a new point set.
			if ((currTimestamp_0 - state->lastFrameTimestamp) >= state->settings.captureDelay) {
				state->lastFrameTimestamp = currTimestamp_0;

				//bool foundPoint = multicalibration_findMarkers(state->cpp_class, caerFrameIteratorElement);
				bool foundPoint = false;
				caerLog(CAER_LOG_WARNING, moduleData->moduleSubSystemString,
					"Searching for markers in the aruco set, result = %d.", foundPoint);
			}
		CAER_FRAME_ITERATOR_VALID_END

		CAER_FRAME_ITERATOR_VALID_START(frame_1)
			uint64_t currTimestamp_1 = U64T(caerFrameEventGetTSStartOfFrame64(caerFrameIteratorElement, frame_1));

		CAER_FRAME_ITERATOR_VALID_END



	}

    // update settings
    updateSettings(moduleData);

}
