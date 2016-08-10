#ifndef MULTIESTIMATION_SETTINGS_H_
#define MULTIESTIMATION_SETTINGS_H_


struct MultiCalibrationSettings_struct {
	bool doCalibration;
	bool doSavetxt;
	char *saveFileName;
	uint32_t captureDelay;
	char *loadFileName_cam0;
	char *loadFileName_cam1;
};

typedef struct MultiCalibrationSettings_struct *MultiCalibrationSettings;


#endif /* MULTIESTIMATION_SETTINGS_H_ */
