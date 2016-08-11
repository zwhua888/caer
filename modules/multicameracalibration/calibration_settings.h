#ifndef MULTIESTIMATION_SETTINGS_H_
#define MULTIESTIMATION_SETTINGS_H_


struct MultiCalibrationSettings_struct {
	bool doCalibration;
	bool doSavetxt;
	char *saveFileName;
	uint32_t captureDelay;
	char *loadFileNames;
	int nCamera;
	int patternWidth;
	int patternHeight;
	int showFeatureExtraction;
	int verbose;
	int nMiniMatches;
	int cameraType;
};

typedef struct MultiCalibrationSettings_struct *MultiCalibrationSettings;


#endif /* MULTICALIBRATION__SETTINGS_H_ */
