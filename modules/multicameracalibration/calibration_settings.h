#ifndef MULTIESTIMATION_SETTINGS_H_
#define MULTIESTIMATION_SETTINGS_H_


struct MultiCalibrationSettings_struct {
	bool detectMarkers;
	bool doSavetxt;
	char *saveFileName;
	uint32_t captureDelay;
	char *loadFileName;
	bool doCalibrateStick;
	char *saveFileNameCalibStick;
	double rejectDistance;
};

typedef struct MultiCalibrationSettings_struct *MultiCalibrationSettings;


#endif /* MULTIESTIMATION_SETTINGS_H_ */
