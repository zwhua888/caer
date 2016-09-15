#include "calibration.hpp"
#include "calibration_wrapper.h"

MultiCalibration *multicalibration_init(MultiCalibrationSettings settings) {
	try {
		return (new MultiCalibration(settings));
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "multiCalibration()",
				"Failed with C++ exception: %s", ex.what());
		return (NULL);
	}
}

void multicalibration_destroy(MultiCalibration *calibClass) {
	try {
		delete calibClass;
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "multiCalibration_destroy()",
				"Failed with C++ exception: %s", ex.what());
	}
}

void multicalibration_updateSettings(MultiCalibration *calibClass) {

}

void multicalibration_freeStereoVec(void * vec1, void * vec2) {

	vector<Point2f> *tmp1, *tmp2;
	tmp1 = ((vector<Point2f>*) vec1);
	tmp2 = ((vector<Point2f>*) vec2);

	if (tmp1 != NULL)
		delete tmp1;
	if (tmp2 != NULL)
		delete tmp2;

	return;
}

void multicalibration_clearImagePoints(MultiCalibration *calibClass){

	try {
			calibClass->clearImagePoints();
		} catch (const std::exception& ex) {
			caerLog(CAER_LOG_ERROR, "multicalibration_clearImagePoints()",
					"Failed with C++ exception: %s", ex.what());
		}

}

void multicalibration_addStereoCalibVec(MultiCalibration *calibClass,
		void * vec1, void * vec2) {

	vector<Point2f> *tmp1, *tmp2;
	tmp1 = ((vector<Point2f>*) vec1);
	tmp2 = ((vector<Point2f>*) vec2);

	try {
		calibClass->addStereoCalib(tmp1, tmp2);
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "multicalibration_addStereoCalibVec()",
				"Failed with C++ exception: %s", ex.what());
	}

}

void * multicalibration_findNewPoints(MultiCalibration *calibClass,
		caerFrameEvent frame, int camid) {
	try {
		return (calibClass->findNewPoints(frame, camid));
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "multiCalibration_multicalib()",
				"Failed with C++ exception: %s", ex.what());
		return (false);
	}
}

bool multicalibration_stereoCalibrate(MultiCalibration *calibClass,
		MultiCalibrationSettings settings) {
	try {
		return (calibClass->stereoCalibrate(settings));
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "multiCalibration_stereoCalibrate()",
				"Failed with C++ exception: %s", ex.what());
		return (false);
	}
}

bool multicalibration_loadCalibrationFile(MultiCalibration *calibClass,
		MultiCalibrationSettings settings) {
	try {
		return (calibClass->loadCalibrationFile(settings));
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "multiCalibration_loadCalibrationFile()",
				"Failed with C++ exception: %s", ex.what());
		return (false);
	}
}

