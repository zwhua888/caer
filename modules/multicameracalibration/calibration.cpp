#include "calibration.hpp"
#include <fstream>
#include <iostream>

MultiCalibration::MultiCalibration(MultiCalibrationSettings settings) {
    this->settings = settings;

    updateSettings(this->settings);
}

void MultiCalibration::updateSettings(MultiCalibrationSettings settings) {
    this->settings = settings;
    
	if (settings->useFisheyeModel) {
		// The fisheye model has its own enum, so overwrite the flags.
		flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC | fisheye::CALIB_FIX_K2
			| fisheye::CALIB_FIX_K3 | fisheye::CALIB_FIX_K4;
	}
	else {
		flag = CALIB_FIX_K4 | CALIB_FIX_K5;

		if (settings->aspectRatio) {
			flag |= CALIB_FIX_ASPECT_RATIO;
		}

		if (settings->assumeZeroTangentialDistortion) {
			flag |= CALIB_ZERO_TANGENT_DIST;
		}

		if (settings->fixPrincipalPointAtCenter) {
			flag |= CALIB_FIX_PRINCIPAL_POINT;
		}
	}

	// Update board size.
	boardSize.width = settings->boardWidth;
	boardSize.height = settings->boardHeigth;

    // Load calibration files keep this in the constructor
    if(loadCalibrationFile(this->settings)){
        this->calibrationLoaded = true;
    }else{
        this->calibrationLoaded = false;
    }

	// Clear current image points.
	imagePoints.clear();

}

bool MultiCalibration::multicalib(caerFrameEvent frame0, caerFrameEvent frame1){
	if (frame0 == NULL || frame1 == NULL || !caerFrameEventIsValid(frame0) || !caerFrameEventIsValid(frame1)
			|| this->calibrationLoaded == false ) {
        caerLog(CAER_LOG_NOTICE, "Multi Calibration multicalib", "Camera matrix and distorsion coefficients not loaded, exit from filter!");
	}
}

bool MultiCalibration::findNewPoints(caerFrameEvent frame) {
	if (frame == NULL || !caerFrameEventIsValid(frame)) {
		return (false);
	}

	// Initialize OpenCV Mat based on caerFrameEvent data directly (no image copy).
	Size frameSize(caerFrameEventGetLengthX(frame), caerFrameEventGetLengthY(frame));
	Mat orig(frameSize, CV_16UC(caerFrameEventGetChannelNumber(frame)), caerFrameEventGetPixelArrayUnsafe(frame));

	// Create a new Mat that has only 8 bit depth from the original 16 bit one.
	// findCorner functions in OpenCV only support 8 bit depth.
	Mat view;
	orig.convertTo(view, CV_8UC(orig.channels()), 1.0 / 256.0);

	int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

	if (!settings->useFisheyeModel) {
		// Fast check erroneously fails with high distortions like fisheye lens.
		chessBoardFlags |= CALIB_CB_FAST_CHECK;
	}

	// Find feature points on the input image.
	vector<Point2f> pointBuf;
	bool found;

	switch (settings->calibrationPattern) {
		case CAMCALIB_CHESSBOARD:
			found = findChessboardCorners(view, boardSize, pointBuf, chessBoardFlags);
			break;

		case CAMCALIB_CIRCLES_GRID:
			found = findCirclesGrid(view, boardSize, pointBuf);
			break;

		case CAMCALIB_ASYMMETRIC_CIRCLES_GRID:
			found = findCirclesGrid(view, boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
			break;

		default:
			found = false;
			break;
	}

	if (found) {
		// Improve the found corners' coordinate accuracy for chessboard pattern.
		if (settings->calibrationPattern == CAMCALIB_CHESSBOARD) {
			Mat viewGray;

			// Only convert color if not grayscale already.
			if (view.channels() == GRAYSCALE) {
				viewGray = view;
			}
			else {
				if (view.channels() == RGB) {
					cvtColor(view, viewGray, COLOR_RGB2GRAY);
				}
				else if (view.channels() == RGBA) {
					cvtColor(view, viewGray, COLOR_RGBA2GRAY);
				}
			}

			cornerSubPix(viewGray, pointBuf, Size(5, 5), Size(-1, -1),
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		}

		imagePoints.push_back(pointBuf);
	}

	return (found);
}

size_t MultiCalibration::foundPoints(void) {
	return (imagePoints.size());
}

bool MultiCalibration::loadCalibrationFile(MultiCalibrationSettings settings) {

	// Open file with undistort matrices.
	/*FileStorage fs(settings->loadFileNames, FileStorage::READ);
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

	 */
	return (true);
}

