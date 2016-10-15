/*
 * datasynch.h
 *
 *  Created on: Oct, 2016
 *      Author: federico.corradi@inilabs.com
 */

#ifndef DATASYNCH_H_
#define DATASYNCH_H_

#include "main.h"

//all events type
#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>
#include <libcaer/events/imu6.h>
#include <libcaer/events/special.h>

void caerDataSynchFilter(uint16_t moduleID, caerPolarityEventPacket polarity_cam0,
		caerPolarityEventPacket polarity_cam1,
		caerFrameEventPacket frame_cam0, caerFrameEventPacket frame_cam1,
		caerIMU6EventPacket imu_cam0, caerIMU6EventPacket imu_cam1,
		caerSpecialEventPacket special_cam0, caerSpecialEventPacket special_cam1
	);

#endif /* DATASYNCH_H_ */
