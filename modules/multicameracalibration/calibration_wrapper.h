#ifndef CALIBRATION_WRAPPER_H_
#define CALIBRATION_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "calibration_settings.h"

typedef struct MultiCalibration MultiCalibration;

MultiCalibration *multicalibration_init(MultiCalibrationSettings settings);
void multicalibration_destroy(MultiCalibration *calibClass);
void multicalibration_updateSettings(MultiCalibration *calibClass);
void * multicalibration_findNewPoints(MultiCalibration *calibClass,
		caerFrameEvent frame, int camid);
void multicalibration_freeStereoVec(void * vec1, void * vec2);
bool multicalibration_stereoCalibrate(MultiCalibration *calibClass,
		MultiCalibrationSettings settings);
void multicalibration_addStereoCalibVec(MultiCalibration *calibClass,
		void * vec1, void * vec2);
bool multicalibration_loadCalibrationFile(MultiCalibration *calibClass,
		MultiCalibrationSettings settings);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_WRAPPER_H_ */
