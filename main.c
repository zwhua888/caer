/*
 * main.c
 *
 *  Created on: Oct 6, 2013
 *      Author: chtekk
 */

#include "main.h"
#include "base/config.h"
#include "base/config_server.h"
#include "base/log.h"
#include "base/mainloop.h"
#include "base/misc.h"

// Devices support.
#ifdef DAVISFX3
	#include "modules/ini/davis_fx3.h"
#endif

// Input/Output support.
#ifdef ENABLE_FILE_INPUT
	#include "modules/misc/in/file.h"
#endif

#ifdef ENABLE_FILE_OUTPUT
	#include "modules/misc/out/file.h"
#endif

// Common filters support.
#ifdef ENABLE_BAFILTER
	#include "modules/backgroundactivityfilter/backgroundactivityfilter.h"
#endif
#ifdef ENABLE_CAMERACALIBRATION
	#include "modules/cameracalibration/cameracalibration.h"
#endif
#ifdef ENABLE_FRAMEENHANCER
	#include "modules/frameenhancer/frameenhancer.h"
#endif
#ifdef ENABLE_STATISTICS
	#include "modules/statistics/statistics.h"
#endif
#ifdef ENABLE_VISUALIZER
	#include "modules/visualizer/visualizer.h"
#endif

#ifdef ENABLE_IMAGEGENERATOR
#include "modules/imagegenerator/imagegenerator.h"
#define MAX_IMG_QTY 8
#define CLASSIFYSIZE 346
#define DISPLAYIMGSIZE 256
#endif


static bool mainloop_1(void);
static bool mainloop_twocameras(void);

static bool mainloop_1(void) {
	// An eventPacketContainer bundles event packets of different types together,
	// to maintain time-coherence between the different events.
	caerEventPacketContainer container = NULL;
	caerSpecialEventPacket special = NULL;
	caerPolarityEventPacket polarity = NULL;

#ifdef ENABLE_VISUALIZER
	caerVisualizerEventHandler visualizerEventHandler = NULL;
#endif

	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
#ifdef DVS128
	container = caerInputDVS128(1);

	// Typed EventPackets contain events of a certain type.
	special = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container, SPECIAL_EVENT);
	polarity = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container, POLARITY_EVENT);
#endif

#ifdef DAVISFX2
	container = caerInputDAVISFX2(1);
#endif
#ifdef DAVISFX3
	container = caerInputDAVISFX3(1);
#endif
#if defined(DAVISFX2) || defined(DAVISFX3)
	// Typed EventPackets contain events of a certain type.
	special = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container, SPECIAL_EVENT);
	polarity = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container, POLARITY_EVENT);

	// Frame and IMU events exist only with DAVIS cameras.
	caerFrameEventPacket frame = NULL;
	caerIMU6EventPacket imu = NULL;

	frame = (caerFrameEventPacket) caerEventPacketContainerGetEventPacket(container, FRAME_EVENT);
	imu = (caerIMU6EventPacket) caerEventPacketContainerGetEventPacket(container, IMU6_EVENT);
#endif

#ifdef ENABLE_FILE_INPUT
	container = caerInputFile(10);
#endif
#ifdef ENABLE_NETWORK_INPUT
	container = caerInputNetTCP(11);
#endif
#if defined(ENABLE_FILE_INPUT) || defined(ENABLE_NETWORK_INPUT)
#ifdef ENABLE_VISUALIZER
	visualizerEventHandler = &caerInputVisualizerEventHandler;
#endif

	// Typed EventPackets contain events of a certain type.
	// We search for them by type here, because input modules may not have all or any of them.
	special = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPECIAL_EVENT);
	polarity = (caerPolarityEventPacket) caerEventPacketContainerFindEventPacketByType(container, POLARITY_EVENT);

	caerFrameEventPacket frame = NULL;
	caerIMU6EventPacket imu = NULL;

	frame = (caerFrameEventPacket) caerEventPacketContainerFindEventPacketByType(container, FRAME_EVENT);
	imu = (caerIMU6EventPacket) caerEventPacketContainerFindEventPacketByType(container, IMU6_EVENT);
#endif

	// Filters process event packets: for example to suppress certain events,
	// like with the Background Activity Filter, which suppresses events that
	// look to be uncorrelated with real scene changes (noise reduction).
#ifdef ENABLE_BAFILTER
	caerBackgroundActivityFilter(2, polarity);
#endif

	// Filters can also extract information from event packets: for example
	// to show statistics about the current event-rate.
#ifdef ENABLE_STATISTICS
	caerStatistics(3, (caerEventPacketHeader) polarity, 1000);
#endif

	// Enable APS frame image enhancements.
#ifdef ENABLE_FRAMEENHANCER
	frame = caerFrameEnhancer(4, frame);
#endif

	// Enable image and event undistortion by using OpenCV camera calibration.
#ifdef ENABLE_CAMERACALIBRATION
	caerCameraCalibration(5, polarity, frame);
#endif

	//Enable camera pose estimation
#ifdef ENABLE_POSEESTIMATION
	caerPoseCalibration(6, polarity, frame);
#endif

	// A simple visualizer exists to show what the output looks like.
#ifdef ENABLE_VISUALIZER
	caerVisualizer(60, "Polarity", &caerVisualizerRendererPolarityEvents, visualizerEventHandler, (caerEventPacketHeader) polarity);
	caerVisualizer(61, "Frame", &caerVisualizerRendererFrameEvents, visualizerEventHandler, (caerEventPacketHeader) frame);
	caerVisualizer(62, "IMU6", &caerVisualizerRendererIMU6Events, visualizerEventHandler, (caerEventPacketHeader) imu);

	//caerVisualizerMulti(68, "PolarityAndFrame", &caerVisualizerMultiRendererPolarityAndFrameEvents, visualizerEventHandler, container);
#endif

#ifdef ENABLE_FILE_OUTPUT
	// Enable output to file (AEDAT 3.X format).
	caerOutputFile(7, 4, polarity, frame, imu, special);
#endif

#ifdef ENABLE_NETWORK_OUTPUT
	// Send polarity packets out via TCP. This is the server mode!
	// External clients connect to cAER, and we send them the data.
	// WARNING: slow clients can dramatically slow this and the whole
	// processing pipeline down!
	caerOutputNetTCPServer(8, 4, polarity, frame, imu, special);

	// And also send them via UDP. This is fast, as it doesn't care what is on the other side.
	caerOutputNetUDP(9, 4, polarity, frame, imu, special);
#endif

#ifdef ENABLE_IMAGEGENERATOR
	// save images of accumulated spikes and frames
	int CLASSIFY_IMG_SIZE = CLASSIFYSIZE;
	char mainString[16] = "_main.c_";

	/* class_region_sizes:
	 * (Not used so far)
	 *
	 * So far we only classify big faces in the center, that cover almost the whole screen.
	 * Instead of classifying only a down-scaled version of the whole quadratic image (created in imagegenerator.c),
	 * we might want to cut out some smaller quadratic windows, downscale them to CLASSIFY_IMAGE_SIZE
	 * and check whether a face is recognized in that smaller area.
	 *
	 * class_region_sizes would store the sizes of this cut out sub-images / windows.
	 * Could be used to display a different frame based on how big the face was, that was recognized)
	 * (assuming only faces in the center are relevant. If other faces should be classified to, one
	 * has to add an array with position of face)
	 */
	int * class_region_sizes = calloc(sizeof(int), MAX_IMG_QTY);
	if (class_region_sizes == NULL) {
		caerLog(CAER_LOG_ERROR, mainString, "Failed to allocate class_region_sizes.");
		return (false);
	}

	/* classification_results:
	 *
	 * Stores the result of the classification (true = FACE, false = NON FACE) for each
	 * element in file_strings_classify
	 */
	double * classification_results = calloc(sizeof(double), MAX_IMG_QTY);
	if (classification_results == NULL) {
		caerLog(CAER_LOG_ERROR, mainString, "Failed to allocate classification_results.");
		return (false);
	}

	/* file_strings_classify:
	 *
	 * Stores all disk locations of images, which we want to classify (so far only one
	 * classify-image per spike-image is generated and put on file_strings_classify[0])
	 */
	unsigned char ** file_strings_classify = calloc(sizeof(unsigned char *), MAX_IMG_QTY);
	if (file_strings_classify == NULL) {
		caerLog(CAER_LOG_ERROR, mainString, "Failed to allocate file_strings_classify.");
		return (false);
	}

	/* display_img_ptr:
	 *
	 * points to the memory location of th image that will be displayed by
	 * the imageStreamerVisualizer
	 */
	unsigned char ** display_img_ptr = calloc(sizeof(unsigned char *), 1);
	if (display_img_ptr == NULL) {
		caerLog(CAER_LOG_ERROR, mainString, "Failed to allocate display_img_ptr.");
		return (false);
	}

	// create image streamer histogram/frame packet
	caerFrameEventPacket imagestreamer = NULL;
	caerFrameEventPacket imagestreamer_frame = NULL;

#if defined(DAVISFX2) || defined(DAVISFX3) || defined(ENABLE_FILE_INPUT) || defined(ENABLE_NETWORK_INPUT)
	unsigned char ** frame_img_ptr = calloc(sizeof(unsigned char *), 1);
	// generate images
	caerImageGenerator(20, polarity, file_strings_classify, (int) MAX_IMG_QTY, CLASSIFY_IMG_SIZE, display_img_ptr, frame, &imagestreamer, &imagestreamer_frame, frame_img_ptr);
#else
	caerImageGenerator(20, polarity, file_strings_classify, (int) MAX_IMG_QTY, CLASSIFY_IMG_SIZE, display_img_ptr, NULL, &imagestreamer, NULL, NULL,);
#endif
#endif

#ifdef ENABLE_CAFFEINTERFACE
	// this also requires image generator
#ifdef ENABLE_IMAGEGENERATOR
	// this wrapper let you interact with caffe framework
	// for example, we now classify the latest image
	// only run CNN if we have a file to classify

	// create image describing network activity
	caerFrameEventPacket networkActivity = NULL;

	if(*file_strings_classify != NULL) {
		caerCaffeWrapper(21, file_strings_classify, classification_results, (int) MAX_IMG_QTY, &networkActivity);
	}
#endif
#endif

#if defined(ENABLE_VISUALIZER) && defined(ENABLE_IMAGEGENERATOR)
	caerVisualizer(65, "ImageStreamerHist", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) imagestreamer);
#if defined(DAVISFX2) || defined(DAVISFX3) || defined(ENABLE_FILE_INPUT) || defined(ENABLE_NETWORK_INPUT)
	// Open a second window of the visualizer
	// display images of accumulated spikes
	// this also requires imagegenerator
	caerVisualizer(64, "ImageStreamerFrame", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) imagestreamer_frame);
#ifdef ENABLE_CAFFEINTERFACE
	//show the activations of the deep network
	caerVisualizer(66, "DeepNetworkActivations", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) networkActivity);
#endif
#endif
#endif

#ifdef ENABLE_IMAGESTREAMERBEEPER
	// add allegro sound on detection
#ifdef ENABLE_CAFFEINTERFACE
	if(classification_results != NULL) {
		caerImagestreamerBeeper(22, classification_results, (int) MAX_IMG_QTY);
	}
#endif
#endif

#ifdef ENABLE_IMAGEGENERATOR
	//free all used data structures
	free(class_region_sizes);
	class_region_sizes = NULL;
	free(classification_results);
	classification_results = NULL;

	for (int i = 1; i < MAX_IMG_QTY; ++i) {
		if (file_strings_classify[i]!= NULL) {
			free(file_strings_classify[i]);
			file_strings_classify[i] = NULL;
		}
	}

	free(file_strings_classify);
	file_strings_classify = NULL;
	free(*display_img_ptr);
	*display_img_ptr = NULL;
	free(display_img_ptr);
	display_img_ptr = NULL;
	free(*frame_img_ptr);
	*frame_img_ptr = NULL;
	free(frame_img_ptr);
	frame_img_ptr = NULL;
	free(imagestreamer);
	free(imagestreamer_frame);
#ifdef ENABLE_CAFFEINTERFACE
	free(networkActivity); // frame that plots network outputs
#endif
#endif

	return (true); // If false is returned, processing of this loop stops.
}

static bool mainloop_twocameras(void) {
	// An eventPacketContainer bundles event packets of different types together,
	// to maintain time-coherence between the different events.

	caerEventPacketContainer container_cam0 = NULL;
	caerSpecialEventPacket special_cam0 = NULL;
	caerPolarityEventPacket polarity_cam0 = NULL;
	caerFrameEventPacket frame_cam0 = NULL;
	caerIMU6EventPacket imu_cam0 = NULL;

	caerEventPacketContainer container_cam1 = NULL;
	caerSpecialEventPacket special_cam1 = NULL;
	caerPolarityEventPacket polarity_cam1 = NULL;
	caerFrameEventPacket frame_cam1 = NULL;
	caerIMU6EventPacket imu_cam1 = NULL;

#ifdef DAVISFX3
	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
	container_cam0 = caerInputDAVISFX3(1);

	// Typed EventPackets contain events of a certain type.
	special_cam0 = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container_cam0, SPECIAL_EVENT);
	polarity_cam0 = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container_cam0, POLARITY_EVENT);
	frame_cam0 = (caerFrameEventPacket) caerEventPacketContainerGetEventPacket(container_cam0, FRAME_EVENT);
	imu_cam0 = (caerIMU6EventPacket) caerEventPacketContainerGetEventPacket(container_cam0, IMU6_EVENT);

	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
	container_cam1 = caerInputDAVISFX3(2);

	// Typed EventPackets contain events of a certain type.
	special_cam1 = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container_cam1, SPECIAL_EVENT);
	polarity_cam1 = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container_cam1, POLARITY_EVENT);
	frame_cam1 = (caerFrameEventPacket) caerEventPacketContainerGetEventPacket(container_cam1, FRAME_EVENT);
	imu_cam1 = (caerIMU6EventPacket) caerEventPacketContainerGetEventPacket(container_cam1, IMU6_EVENT);
#endif

#ifdef ENABLE_FILE_INPUT
	container_cam0 = caerInputFile(10);

	// Typed EventPackets contain events of a certain type.
	// We search for them by type here, because input modules may not have all or any of them.
	special_cam0 = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam0, SPECIAL_EVENT);
	polarity_cam0 = (caerPolarityEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam0, POLARITY_EVENT);
	frame_cam0 = (caerFrameEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam0, FRAME_EVENT);
	imu_cam0 = (caerIMU6EventPacket) caerEventPacketContainerFindEventPacketByType(container_cam0, IMU6_EVENT);

	container_cam1 = caerInputFile(10);

	// Typed EventPackets contain events of a certain type.
	// We search for them by type here, because input modules may not have all or any of them.
	special_cam1 = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam1, SPECIAL_EVENT);
	polarity_cam1 = (caerPolarityEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam1, POLARITY_EVENT);
	frame_cam1 = (caerFrameEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam1, FRAME_EVENT);
	imu_cam1 = (caerIMU6EventPacket) caerEventPacketContainerFindEventPacketByType(container_cam1, IMU6_EVENT);
#endif

	// Filters process event packets: for example to suppress certain events,
	// like with the Background Activity Filter, which suppresses events that
	// look to be uncorrelated with real scene changes (noise reduction).
#ifdef ENABLE_BAFILTER
	caerBackgroundActivityFilter(2, polarity_cam0);
	caerBackgroundActivityFilter(22, polarity_cam1);
#endif

	// Enable APS frame image enhancements.
#ifdef ENABLE_FRAMEENHANCER
	frame_0 = caerFrameEnhancer(4, frame_cam0);
	frame_1 = caerFrameEnhancer(44, frame_cam1);
#endif

	// Enable image and event undistortion by using OpenCV camera calibration.
#ifdef ENABLE_CAMERACALIBRATION
	caerCameraCalibration(5, polarity_cam0, frame_cam0);
	caerCameraCalibration(55, polarity_cam1, frame_cam1);
#endif


#ifdef ENABLE_MULTICAMERACALIBRATION
	caerMultiCalibration(7, frame_cam0, frame_cam1);
#endif


	// A simple visualizer exists to show what the output looks like.
#ifdef ENABLE_VISUALIZER
	caerVisualizerMulti(68, "PolarityAndFrame", &caerVisualizerMultiRendererPolarityAndFrameEvents, NULL, container_cam0);
	caerVisualizerMulti(6688, "PolarityAndFrame", &caerVisualizerMultiRendererPolarityAndFrameEvents, NULL, container_cam1);
#if defined(ENABLE_VISUALIZER) && defined(ENABLE_IMAGEGENERATOR)
	caerVisualizer(64, "ImageStreamerFrameCam0", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) frame_cam0);
	caerVisualizer(6644, "ImageStreamerFrameCam0", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) frame_cam1);
#endif
#endif


#ifdef ENABLE_FILE_OUTPUT
	// Enable output to file (AER3.1 format).
	caerOutputFile(9, 4, polarity_cam0, frame_cam0, imu_cam0, special_cam0);
	caerOutputFile(99, 4, polarity_cam1, frame_cam1, imu_cam1, special_cam1);
#endif

	return (true); // If false is returned, processing of this loop stops.
}

int main(int argc, char **argv) {
	// Initialize config storage from file, support command-line overrides.
	// If no init from file needed, pass NULL.
	caerConfigInit("caer-config.xml", argc, argv);

	// Initialize logging sub-system.
	caerLogInit();

	// Initialize visualizer framework (load fonts etc.).
#ifdef ENABLE_VISUALIZER
	caerVisualizerSystemInit();
#endif

	// Daemonize the application (run in background).
	//caerDaemonize();

	// Start the configuration server thread for run-time config changes.
	caerConfigServerStart();

	// Finally run the main event processing loops.
	struct caer_mainloop_definition mainLoops[1] = { { 1, &mainloop_twocameras } };
	caerMainloopRun(&mainLoops, 1);

	// After shutting down the mainloops, also shutdown the config server
	// thread if needed.
	caerConfigServerStop();

	return (EXIT_SUCCESS);
}
