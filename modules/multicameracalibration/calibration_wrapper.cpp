#include "calibration.hpp"
#include "calibration_wrapper.h"

MultiCalibration *multicalibration_init(MultiCalibrationSettings settings) {
	try {
		return (new MultiCalibration(settings));
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "multiCalibration()", "Failed with C++ exception: %s", ex.what());
		return (NULL);
	}
}


void multicalibration_destroy(MultiCalibration *calibClass) {
	try {
		delete calibClass;
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "multiCalibration_destroy()", "Failed with C++ exception: %s", ex.what());
	}
}

void multicalibration_updateSettings(MultiCalibration *calibClass) {
	
}

bool multicalibration_findNewPoints(MultiCalibration *calibClass, caerFrameEvent frame) {
	try {
		return (calibClass->findNewPoints(frame));
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "multiCalibration_multicalib()", "Failed with C++ exception: %s", ex.what());
		return (false);
	}
}

bool multicalibration_multicalib(MultiCalibration *calibClass, caerFrameEvent frame0, caerFrameEvent frame1) {
	try {
		return (calibClass->multicalib(frame0, frame1));
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "multiCalibration_multicalib()", "Failed with C++ exception: %s", ex.what());
		return (false);
	}
}

bool multicalibration_loadCalibrationFile(MultiCalibration *calibClass, MultiCalibrationSettings settings) {
	try {
		return (calibClass->loadCalibrationFile(settings));
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "multiCalibration_loadCalibrationFile()", "Failed with C++ exception: %s", ex.what());
		return (false);
	}
}

