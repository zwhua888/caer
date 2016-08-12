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
bool multicalibration_findNewPoints(MultiCalibration *calibClass, caerFrameEvent frame);
bool multicalibration_multicalib(MultiCalibration *calibClass, caerFrameEvent frame0, caerFrameEvent frame1);
bool multicalibration_loadCalibrationFile(MultiCalibration *calibClass, MultiCalibrationSettings settings);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_WRAPPER_H_ */
