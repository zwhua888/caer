#ifndef MULTIESTIMATION_SETTINGS_H_
#define MULTIESTIMATION_SETTINGS_H_

enum CameraCalibrationPattern { CAMCALIB_CHESSBOARD, CAMCALIB_CIRCLES_GRID, CAMCALIB_ASYMMETRIC_CIRCLES_GRID };

struct MultiCalibrationSettings_struct {
	bool doCalibration;
	bool doSavetxt;
	char *saveFileName;
	uint32_t captureDelay;
	char *loadFileNames;
	int nCamera;
	uint32_t boardWidth;
	uint32_t boardHeigth;
	int patternWidth;
	int patternHeight;
	int showFeatureExtraction;
	int verbose;
	int nMiniMatches;
	int cameraType;
	bool useFisheyeModel;
	enum CameraCalibrationPattern calibrationPattern;
	float aspectRatio;
	bool assumeZeroTangentialDistortion;
	bool fixPrincipalPointAtCenter;

};

typedef struct MultiCalibrationSettings_struct *MultiCalibrationSettings;


#endif /* MULTICALIBRATION__SETTINGS_H_ */
