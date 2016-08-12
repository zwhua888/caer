#ifndef MULTICALIBRATION_HPP_
#define MULTICALIBRATION_HPP_

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include "calibration_settings.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

class MultiCalibration {

public:
    MultiCalibration(MultiCalibrationSettings settings);
	bool findNewPoints(caerFrameEvent frame);
	size_t foundPoints(void);
    bool multicalib(caerFrameEvent frame0, caerFrameEvent frame1);
    bool loadCalibrationFile(MultiCalibrationSettings settings);
    void updateSettings(MultiCalibrationSettings settings);
    Point3f getCamerasLocation(Mat Rvec,Mat Tvec);

private:
    MultiCalibrationSettings settings = NULL;
	Size boardSize;
	int flag = 0; //fish eye model
	vector<vector<Point2f> > imagePoints;
    Mat undistortCameraMatrix;
    bool useFisheyeModel;
    Mat undistortDistCoeffs;
    double focal_lenght_mm = 6;
    int camera_x_resolution = 260;
    int camera_y_resolution = 346;
    double object_real_world_mm = 15; // obejct is 40 mm
    bool calibrationLoaded = false;

};

#endif /* MULTICALIBRATION_HPP_ */
