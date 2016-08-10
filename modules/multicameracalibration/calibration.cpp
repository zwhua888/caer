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
        this->calibrationLoaded_cam0 = true;
        this->calibrationLoaded_cam1 = true;
    }else{
        this->calibrationLoaded_cam0 = false;
        this->calibrationLoaded_cam1 = false;
    }

}

bool MultiCalibration::findMarkers(caerFrameEvent frame) {
    if (frame == NULL || !caerFrameEventIsValid(frame) || this->calibrationLoaded_cam0 == false) {
        if(this->calibrationLoaded_cam0 == false){
            caerLog(CAER_LOG_NOTICE, "Multi Calibration findMarkers", "Camera matrix and distorsion coefficients not loaded, exit from filter!");
        }
	    return (false);
    }

    // Initialize OpenCV Mat based on caerFrameEvent data directly (no image copy).
    Size frameSize(caerFrameEventGetLengthX(frame), caerFrameEventGetLengthY(frame));
    Mat orig(frameSize, CV_16UC(caerFrameEventGetChannelNumber(frame)), caerFrameEventGetPixelArrayUnsafe(frame));

    double ts_frame = caerFrameEventGetTimestamp(frame);

    // Create a new Mat that has only 8 bit depth from the original 16 bit one.
    // findCorner functions in OpenCV only support 8 bit depth.
    Mat view;
    orig.convertTo(view, CV_8UC(orig.channels()), 1.0 / 256.0);
    
    // marker detection with aruco markers integration in opencv
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
    

    vector<int> ids; 
    vector<std::vector<cv::Point2f> > corners, rejected; 
    
    aruco::DetectorParameters * detectorParams = new aruco::DetectorParameters;
    detectorParams->doCornerRefinement = true; // do corner refinement in markers

    // detect
    aruco::detectMarkers(view, dictionary, corners, ids, *detectorParams, rejected);
    cv::aruco::Board board =  aruco::GridBoard::create(7,5,15,3,dictionary);
    
    // if at least one marker has been detected
    if (ids.size() > 0){
        aruco::drawDetectedMarkers(view, corners, ids);
    }else{
        return(false);
    }
       
    // from camera calibration 
    double fx, fy, m, distance, avr_size, x, object_image_sensor_mm;
    fx = undistortCameraMatrix_cam0.at<double>(0,0);
    fy = undistortCameraMatrix_cam0.at<double>(1,1);
    // from zhang method we ervecs tvecs to 3d pointsstimate pixels per mm (focal lenght))
    m = ( (fx+fy)/2.0 ) / focal_lenght_mm_cam0 ;
    // estimate markers Multi
    if( ids.size() > 0){

        Mat rvecs_board, tvecs_board;
    	int valid = cv::aruco::estimatePoseBoard(corners, ids, board, undistortCameraMatrix_cam0, undistortDistCoeffs_cam0, rvecs_board, tvecs_board);
	    caerLog(CAER_LOG_WARNING, "ArucoMultiEstimation ", "Board detected %d\n", valid);

	    //estimate pose board
	    if(valid > 0){
			cv::Point3f coords, rots;

			Mat rvec;
			//rvec = rvecs.col(k).t();
			//cout << rvecs.col(0).reshape(1).t() << endl;
			Mat R;
			cv::Rodrigues(rvecs_board.col(0).reshape(1).t(), R);  // R is 3x3

			R = R.t(); // rotation of inverse
			cv::Mat tvec_cam, rvec_cam;

            // project 3d axis to 2d image plane using rvecs and tvecs
            float length = 0.07;
            vector< Point3f > axisPoints;
            axisPoints.push_back(Point3f(0, 0, 0));
            axisPoints.push_back(Point3f(length, 0, 0));
            axisPoints.push_back(Point3f(0, length, 0));
            axisPoints.push_back(Point3f(0, 0, length));
            vector< Point2f > imagePoints;
            projectPoints(axisPoints, rvecs_board.col(0).reshape(1).t(), tvecs_board.col(0).reshape(1).t(), undistortCameraMatrix_cam0, undistortDistCoeffs_cam0, imagePoints);
            // draw axis lines
            line(view, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
            line(view, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
            line(view, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);

            //estimate distance in mm
            cv::RotatedRect box = cv::minAreaRect(cv::Mat(corners[0]));
            cv::Point2f p = box.size;
            avr_size = ( p.x + p.y ) / 2.0; //in pixels
            // convert px/mm in the lower resolution
            // camera_max_resolution/m = camera_y_resolution/x
            x = (camera_y_resolution_cam0*m)/camera_y_resolution_cam0;
            object_image_sensor_mm = avr_size / x ;
            // calculate distance from object
            // distance_mm = object_real_world_mm * focal-length_mm / object_image_sensor_mm
            distance = object_real_world_mm_cam0 * focal_lenght_mm_cam0 / object_image_sensor_mm;

            // Extract 3x3 rotation matrix
            Mat Rot(3,3,CV_32FC1);
            Rodrigues(rvecs_board.col(0).reshape(1).t(), Rot); // euler angles to rotation matrix

            // Full 4x4 output matrix:
            Mat full(4,4,CV_32FC1);

            // Copy rotation 3x3
            for (int i=0; i<3; i++)
                for (int j=0; j<3; j++)
                    full.at<float>(i,j)=Rot.at<double>(i,j);

            // Copy translation vector
            full.at<float>(0,3)=tvecs_board.col(0).at<double>(0,0);
            full.at<float>(1,3)=tvecs_board.col(0).at<double>(1,0);
            full.at<float>(2,3)=tvecs_board.col(0).at<double>(2,0);

            // Final row is identity (nothing happening on W axis)
            full.at<float>(3,0)=0.0;
            full.at<float>(3,1)=0.0;
            full.at<float>(3,2)=0.0;
            full.at<float>(3,3)=1.0;

            Mat back=full.inv();

            // Splat to screen, for debugging
            /*printf("###\n");
            for (int i=0; i<4; i++) {
                for (int j=0; j<4; j++)
                    printf("%.5f	",full.at<float>(i,j));
                printf("\n");
            }
            printf("###\n");*/


            for(int this_point = 0 ; this_point <= 3; this_point++){

                cv::Mat pointin3df(1, 4, DataType<double>::type);
                pointin3df.at<double>(0,0) = corners[0][this_point].x+1;
                pointin3df.at<double>(0,1) = corners[0][this_point].y+1;
                pointin3df.at<double>(0,2) = 1;
                pointin3df.at<double>(0,3) = 1;
                pointin3df.convertTo(pointin3df,CV_32FC1,1,0);
                back.convertTo(back,CV_32FC1,1,0); //NOW A IS FLOAT
                Mat srcP = Mat(pointin3df).reshape(1).t();
                Mat point_3d_back = back*srcP;
                full.convertTo(full,CV_32FC1,1,0); //NOW A IS FLOAT
                Mat point_3d_full = full*srcP;

                if(settings->doSavetxt){
						ofstream myfile;
						myfile.open (settings->saveFileName, ofstream::out | ofstream::app); //points_3d.t()
						myfile << corners[0][this_point] <<  "\t"  << point_3d_back.t() << "\t"
								<< distance << "\t" << rvecs_board.col(0) << "\t" << tvecs_board.col(0) << "\t" << ts_frame << "\t" << ids[0]
								<<  "\n" << endl;
						myfile.close();
                }
            }

	    }

    }

    //estimate single markers
    Mat rvecs, tvecs;
    aruco::estimatePoseSingleMarkers(corners, 0.05, undistortCameraMatrix_cam0, undistortDistCoeffs_cam0, rvecs, tvecs);

    // rvecs tvecs tell us where the marker is in camera coordinates [R T; 0 1] (for homogenous coordinates)
    // the inverse is [R^t -R^t*T; 0 1]
    for(unsigned int k=0; k<corners.size(); k++){

        // project 3d axis to 2d image plane using rvecs and tvecs
        float length = 0.07;
        vector< Point3f > axisPoints;
        axisPoints.push_back(Point3f(0, 0, 0));
        axisPoints.push_back(Point3f(length, 0, 0));
        axisPoints.push_back(Point3f(0, length, 0));
        axisPoints.push_back(Point3f(0, 0, length));
        vector< Point2f > imagePoints;
        projectPoints(axisPoints, rvecs.row(k), tvecs.row(k), undistortCameraMatrix_cam0, undistortDistCoeffs_cam0, imagePoints);
        // draw axis lines
        line(view, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
        line(view, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
        line(view, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);

        //estimate distance in mm
        cv::RotatedRect box = cv::minAreaRect(cv::Mat(corners[k]));
        cv::Point2f p = box.size;
        avr_size = ( p.x + p.y ) / 2.0; //in pixels
        // convert px/mm in the lower resolution
        // camera_max_resolution/m = camera_y_resolution/x
        x = (camera_y_resolution_cam0*m)/camera_y_resolution_cam0;
        object_image_sensor_mm = avr_size / x ;
        // calculate distance from object
        // distance_mm = object_real_world_mm * focal-length_mm / object_image_sensor_mm
        distance = object_real_world_mm_cam0 * focal_lenght_mm_cam0 / object_image_sensor_mm;

        // Extract 3x3 rotation matrix
        Mat Rot(3,3,CV_32FC1);
        Rodrigues(rvecs.row(k), Rot); // euler angles to rotation matrix

        // Full 4x4 output matrix:
        Mat full(4,4,CV_32FC1);

        // Copy rotation 3x3
        for (int i=0; i<3; i++)
            for (int j=0; j<3; j++)
                full.at<float>(i,j)=Rot.at<double>(i,j);

        // Copy translation vector
        full.at<float>(0,3)=tvecs.row(k).at<double>(0,0);
        full.at<float>(1,3)=tvecs.row(k).at<double>(1,0);
        full.at<float>(2,3)=tvecs.row(k).at<double>(2,0);

        // Final row is identity (nothing happening on W axis)
        full.at<float>(3,0)=0.0;
        full.at<float>(3,1)=0.0;
        full.at<float>(3,2)=0.0;
        full.at<float>(3,3)=1.0;

        // Invert, to convert marker-from-camera into camera-from-marker
        Mat back=full.inv();

        // Splat to screen, for debugging
        /*printf("###\n");
        for (int i=0; i<4; i++) {
            for (int j=0; j<4; j++)
                printf("%.5f	",back.at<float>(i,j));
            printf("\n");
        }
        printf("###\n");*/

        for(int this_point = 0 ; this_point <= 3; this_point++){

            cv::Mat pointin3df(1, 4, DataType<double>::type);
            pointin3df.at<double>(0,0) = corners[k][this_point].x+1;
            pointin3df.at<double>(0,1) = corners[k][this_point].y+1;
            pointin3df.at<double>(0,2) = 1;
            pointin3df.at<double>(0,3) = 1;
            pointin3df.convertTo(pointin3df,CV_32FC1,1,0);
            back.convertTo(back,CV_32FC1,1,0); //NOW A IS FLOAT
            Mat srcP = Mat(pointin3df).reshape(1).t();
            Mat point_3d_back = back*srcP;
            full.convertTo(full,CV_32FC1,1,0); //NOW A IS FLOAT
            Mat point_3d_full = full*srcP;

            // check for nan
        	if( std::isnan(point_3d_back.at<float>(0,1)) ||
        		std::isnan(point_3d_back.at<float>(0,2)) ||
				std::isnan(point_3d_back.at<float>(0,3)) ){
        		continue;
            //check for crazy numbers
        	}else if(settings->doSavetxt){
                ofstream myfile;
                myfile.open (settings->saveFileName, ofstream::out | ofstream::app); //points_3d.t()
                myfile << corners[k][this_point] <<  "\t"  << point_3d_back.t() << "\t"
                        << distance << "\t" << rvecs.row(k) << "\t" << tvecs.row(k) << "\t" << ts_frame << "\t" << ids[k]
						<<  "\n" << endl;
                myfile.close();
            }
         }

    }


    //place back the markers in the frame
    view.convertTo(orig, CV_16UC(orig.channels()), 256.0);

    return(true);

}

bool MultiCalibration::loadCalibrationFile(MultiCalibrationSettings settings) {

	// Open file with undistort matrices.
	FileStorage fs(settings->loadFileName_cam0, FileStorage::READ);
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

	// Open file with undistort matrices.
	FileStorage fss(settings->loadFileName_cam1, FileStorage::READ);
	// Check file.
	if (!fss.isOpened()) {
		return (false);
	}
	fss["camera_matrix"] >> undistortCameraMatrix_cam1;
	fss["distortion_coefficients"] >> undistortDistCoeffs_cam1;
	fss["use_fisheye_model"] >> useFisheyeModel_cam1;
	if (!fs["camera_matrix"].empty() && !fs["distortion_coefficients"].empty()) 
	{
	    caerLog(CAER_LOG_NOTICE, "MultiCalibration CXX loadCalibrationFile()", "Camera matrix and distorsion coefficients succesfully loaded");
	}else{
	    caerLog(CAER_LOG_ERROR, "MultiCalibration CXX loadCalibrationFile()", "Camera matrix and distorsion coefficients not loaded");
	}    
	// Close file.
	fss.release();

	return (true);
}

