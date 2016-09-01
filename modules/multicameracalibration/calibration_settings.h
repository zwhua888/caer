#ifndef MULTIESTIMATION_SETTINGS_H_
#define MULTIESTIMATION_SETTINGS_H_

enum CameraCalibrationPattern { CAMCALIB_CHESSBOARD, CAMCALIB_CIRCLES_GRID, CAMCALIB_ASYMMETRIC_CIRCLES_GRID };

struct MultiCalibrationSettings_struct {
	bool doCalibration;
	char *loadFileName_cam0;
	bool useFisheyeModel_cam0;
	bool useFisheyeModel_cam1;
	char *loadFileName_cam1;
	char *saveFileName_extrinsics;
	char *saveFileName_intrinsics;
	uint32_t captureDelay;
	uint32_t numPairsImagesBeforCalib;
	uint32_t boardWidth;
	uint32_t boardHeigth;
	int patternWidth;
	int patternHeight;
	int showFeatureExtraction;
	int verbose;
	int nMiniMatches;
	int cameraType;
	enum CameraCalibrationPattern calibrationPattern;
	float aspectRatio;
	bool assumeZeroTangentialDistortion;
	bool fixPrincipalPointAtCenter;
	float boardSquareSize;

};

typedef struct MultiCalibrationSettings_struct *MultiCalibrationSettings;


#endif /* MULTICALIBRATION__SETTINGS_H_ */
