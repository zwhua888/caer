#ifndef POSEESTIMATION_H_
#define POSEESTIMATION_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

void caerMultiCalibration(uint16_t moduleID, caerFrameEventPacket frame_0, caerFrameEventPacket frame_1);

#endif /* MULTICALIBRATION_H_ */
