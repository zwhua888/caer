#include "calibration.hpp"
#include <fstream>
#include <iostream>

MultiCalibration::MultiCalibration(MultiCalibrationSettings settings) {
    this->settings = settings;

    updateSettings(this->settings);
}

void MultiCalibration::updateSettings(MultiCalibrationSettings settings) {
    this->settings = settings;
    
    // Load calibration files keep this in the constructor
    if(loadCalibrationFile(this->settings)){
        this->calibrationLoaded = true;
    }else{
        this->calibrationLoaded = false;
    }

}

bool MultiCalibration::multicalib(caerFrameEvent frame0, caerFrameEvent frame1){
	if (frame0 == NULL || frame1 == NULL || !caerFrameEventIsValid(frame0) || !caerFrameEventIsValid(frame1)
			|| this->calibrationLoaded == false ) {
        caerLog(CAER_LOG_NOTICE, "Multi Calibration multicalib", "Camera matrix and distorsion coefficients not loaded, exit from filter!");
	}
}


bool MultiCalibration::loadCalibrationFile(MultiCalibrationSettings settings) {

	// Open file with undistort matrices.
	FileStorage fs(settings->loadFileNames, FileStorage::READ);
	// Check file.
	if (!fs.isOpened()) {
		return (false);
	}
	fs["camera_matrix"] >> undistortCameraMatrix_cam0;
	fs["distortion_coefficients"] >> undistortDistCoeffs_cam0;
	fs["use_fisheye_model"] >> useFisheyeModel_cam0;
	if (!fs["camera_matrix"].empty() && !fs["distortion_coefficients"].empty())
	{
	    caerLog(CAER_LOG_NOTICE, "MultiCalibration CXX loadCalibrationFile()", "Camera matrix and distorsion coefficients succesfully loaded");
	}else{
	    caerLog(CAER_LOG_ERROR, "MultiCalibration CXX loadCalibrationFile()", "Camera matrix and distorsion coefficients not loaded");
	}
	// Close file.
	fs.release();


	return (true);
}

