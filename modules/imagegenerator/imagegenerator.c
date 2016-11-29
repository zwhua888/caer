/*
 *  accumulates a fixed number of events and generates png pictures
 *  it also displays png images when available
 *  federico.corradi@inilabs.com
 */
#include <limits.h>
#include <float.h>
#include "imagegenerator.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "modules/statistics/statistics.h"
#include "ext/portable_time.h"
#include "ext/buffers.h"
#include <string.h>
#include <stdio.h>

#include "main.h"
#include <libcaer/events/polarity.h>

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
/* inlude stblib image library */
#include "ext/stblib/stb_image_write.h"
#include "ext/stblib/stb_image_resize.h"

#define TESTING 0  // keyboard "r" or "t" (recording or testing) "s" (stop) real-time test network, stores images in /tmp/ as defined in header file .h
#define TRAINING_POSITIVES 1 // keyboard "p" (positives) record pngs and store them in positive folder
#define TRAINING_NEGATIVES 2 // keyboard "n" (negatives) record pngs and store them in negative folder
#define FRAME_SAVE_MODE 3       // used for saving frames with the save_img function, the file name contains "_frame_"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

// keyboard "s" stop saving png, generations on visualizer keeps going

//define if you want to save frames to disk into FRAME_IMG_DIRECTORY (for analyzing and debugging)
static bool SAVE_FRAME = false;

struct imagegenerator_state {
    uint32_t *eventRenderer;
    int16_t eventRendererSizeX;
    int16_t eventRendererSizeY;
    struct caer_statistics_state eventStatistics;
    bool rectifyPolarities;
    int colorscale;
    //image matrix
    int64_t **ImageMap;
    int32_t numSpikes; // after how many spikes will we generate an image
    int32_t spikeCounter; // actual number of spikes seen so far, in range [0, numSpikes]
    int32_t counterImg; // how many spikeImages did we produce so far
    int32_t counterTxt;
    int32_t counterFrame; // how many frames did we see so far
    int16_t sizeX;
    int16_t sizeY;
    // frame
    uint16_t *frameRenderer;
    int32_t frameRendererSizeX;
    int32_t frameRendererSizeY;
    int32_t frameRendererPositionX;
    int32_t frameRendererPositionY;
    enum caer_frame_event_color_channels frameChannels;
};

typedef struct imagegenerator_state *imagegeneratorState;

static bool caerImageGeneratorInit(caerModuleData moduleData);
static void caerImageGeneratorRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerImageGeneratorExit(caerModuleData moduleData);
static bool allocateImageMapSubsampled(imagegeneratorState state, int16_t sourceID, int16_t sizeX,
        int16_t sizeY);

static struct caer_module_functions caerImageGeneratorFunctions = { .moduleInit =
        &caerImageGeneratorInit, .moduleRun = &caerImageGeneratorRun, .moduleConfig =
NULL, .moduleExit = &caerImageGeneratorExit };

void caerImageGenerator(uint16_t moduleID, caerPolarityEventPacket polarity, int classify_img_size,
        int *packet_hist, bool * haveimg) {

    caerModuleData moduleData = caerMainloopFindModule(moduleID, "ImageGenerator", PROCESSOR);
    if (moduleData == NULL) {
        return;
    }

    caerModuleSM(&caerImageGeneratorFunctions, moduleData, sizeof(struct imagegenerator_state), 4,
            polarity, classify_img_size, packet_hist, haveimg);

    return;
}

static bool caerImageGeneratorInit(caerModuleData moduleData) {

    // Ensure numSpikes is set.
    imagegeneratorState state = moduleData->moduleState;
    sshsNodePutIntIfAbsent(moduleData->moduleNode, "numSpikes", 2000);
    state->numSpikes = sshsNodeGetInt(moduleData->moduleNode, "numSpikes");

    sshsNodePutBoolIfAbsent(moduleData->moduleNode, "rectifyPolarities", true);
    state->rectifyPolarities = sshsNodeGetBool(moduleData->moduleNode, "rectifyPolarities");

    sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscale", 200);
    state->colorscale = sshsNodeGetInt(moduleData->moduleNode, "colorscale");

    sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
    if (!sshsNodeAttributeExists(sourceInfoNode, "dvsSizeX", SHORT)) {
        sshsNodePutShortIfAbsent(moduleData->moduleNode, "dvsSizeX", 240);
        sshsNodePutShortIfAbsent(moduleData->moduleNode, "dvsSizeY", 180);
    }

    state->ImageMap = NULL;

    return (true);
}

static void caerImageGeneratorExit(caerModuleData moduleData) {
    imagegeneratorState state = moduleData->moduleState;

    // Ensure render maps are freed.
    if (state->eventRenderer != NULL) {
        free(state->eventRenderer);
        state->eventRenderer = NULL;
    }

}

//This function implement 3sigma normalization and converts the image in nullhop format
static bool normalize_image_map_sigma(imagegeneratorState state, int * hist, int size) {

    int sum, count = 0;
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (state->ImageMap[i][j] != 0) {
                sum += state->ImageMap[i][j];
                count++;
            }
        }
    }

    float mean = sum / count;

    float var = 0;
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (state->ImageMap[i][j] != 0) {
                float f = state->ImageMap[i][j] - mean;
                var += f * f;
            }
        }
    }

    float sig = sqrt(var / count);
    if (sig < (0.1f / 255.0f)) {
        sig = 0.1f / 255.0f;
    }

    float numSDevs = 3;
    float mean_png_gray, range, halfrange;

    if (state->rectifyPolarities) {
        mean_png_gray = 0; // rectified
    } else {
        mean_png_gray = (127.0 / 255.0) * 256.0f; //256 included here for nullhop reshift
    }
    if (state->rectifyPolarities) {
        range = numSDevs * sig * (1.0f / 256.0f); //256 included here for nullhop reshift
        halfrange = 0;
    } else {
        range = numSDevs * sig * 2 * (1.0f / 256.0f);
        ; //256 included here for nullhop reshift
        halfrange = numSDevs * sig;
    }

    for (int col_idx = 0; col_idx < size; col_idx++) {
        for (int row_idx = 0; row_idx < size; row_idx++) {

            int linindex = row_idx * size + col_idx; //we have to increase column as fastest idx, then row in order to have a zs compatible system

            if (state->ImageMap[col_idx][row_idx] == 0) {

                hist[linindex] = mean_png_gray;

            } else {
                float f = (state->ImageMap[col_idx][row_idx] + halfrange) / range;

                if (f > 256) {
                    f = 256; //256 included here for nullhop reshift
                } else if (f < 0) {
                    f = 0;
                }

                hist[linindex] = floor(f); //shift by 256 included in previous computations
            }
        }
    }

    return (true);

}

static void caerImageGeneratorRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
    UNUSED_ARGUMENT(argsNumber);

    // Interpret variable arguments (same as above in main function).
    caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
    int CLASSIFY_IMG_SIZE = va_arg(args, int);
    int * hist = va_arg(args, int*);
    bool * haveimg = va_arg(args, bool*);
    haveimg[0] = false;

    // Only process packets with content.
    // what if content is not a polarity event?
    if (polarity == NULL) {
        return;
    }

    //update module state
    imagegeneratorState state = moduleData->moduleState;

    sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");

    /* **** SPIKE SECTION START *** */
    // If the map is not allocated yet, do it.
    if (state->ImageMap == NULL) {
        if (!allocateImageMapSubsampled(state,
                caerEventPacketHeaderGetEventSource(&polarity->packetHeader), CLASSIFY_IMG_SIZE,
                CLASSIFY_IMG_SIZE)) {
            // Failed to allocate memory, nothing to do.
            caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
                    "Failed to allocate memory for ImageMap.");
            return;
        }
    }

    if (polarity != NULL) {

        float cam_sizeX = 240.f;	//#sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
        float cam_sizeY = 180.f;	//#sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

        float res_x = CLASSIFY_IMG_SIZE / cam_sizeX;
        float res_y = CLASSIFY_IMG_SIZE / cam_sizeY;

        if (state->colorscale <= 0) {
            caerLog(CAER_LOG_CRITICAL, "imagegenerator", "please select colorscale >0");
            exit(1);
        }

        caerEventPacketHeader *onlyvalid =  caerCopyEventPacketOnlyValidEvents(polarity);
        if(onlyvalid == NULL){
            return;
        }
        int32_t numtotevs =   caerEventPacketHeaderGetEventValid(onlyvalid);

        caerPolarityEventPacket currPolarityPacket = (caerPolarityEventPacket) onlyvalid;

        //default is all events
        int numevs_start = 0;
        int numevs_end = numtotevs;

        if( numtotevs >= state->numSpikes ){
            //get rid of accumulated spikes
            for (int col_idx = 0; col_idx < state->sizeX; col_idx++) {
                for (int row_idx = 0; row_idx < state->sizeY; row_idx++) {
                    state->ImageMap[col_idx][row_idx] = 0;
                }
            }
            state->spikeCounter = 0;
            //takes only the last 2000
            numevs_start = numtotevs - state->numSpikes;
            numevs_end = numtotevs;

        }else if( (numtotevs+state->spikeCounter) >= state->numSpikes ){
            //takes only the last 2000
            numevs_start = numtotevs - (state->numSpikes-state->spikeCounter);
            numevs_end = numtotevs;
        }


        for (int32_t caerPolarityIteratorCounter = numevs_start;
                caerPolarityIteratorCounter
                        <  numevs_end;
                caerPolarityIteratorCounter++) {

            caerPolarityEvent caerPolarityIteratorElement = caerPolarityEventPacketGetEvent(
                    currPolarityPacket, caerPolarityIteratorCounter);

            if (!caerPolarityEventIsValid(caerPolarityIteratorElement)) {
                continue;
            } // Skip invalid polarity events.

            // Get coordinates and polarity (0 or 1) of latest spike.
            uint16_t x = caerPolarityEventGetX(caerPolarityIteratorElement);
            uint16_t y = caerPolarityEventGetY(caerPolarityIteratorElement);
            int pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);

            uint16_t pos_x = floor(res_x * x);
            uint16_t pos_y = floor(res_y * y);

            //Update image Map
            if (state->rectifyPolarities) {
                state->ImageMap[pos_x][pos_y] = state->ImageMap[pos_x][pos_y] + 1; //rectify events
            } else {
                if (pol == 0) {
                    state->ImageMap[pos_x][pos_y] = state->ImageMap[pos_x][pos_y] - 1;
                } else {
                    state->ImageMap[pos_x][pos_y] = state->ImageMap[pos_x][pos_y] + 1;
                }
            }

            if (state->ImageMap[pos_x][pos_y] > state->colorscale) {
                state->ImageMap[pos_x][pos_y] = state->colorscale;
            } else if (state->ImageMap[pos_x][pos_y] < -state->colorscale) {
                state->ImageMap[pos_x][pos_y] = -state->colorscale;
            }

            state->spikeCounter += 1;

            //If we saw enough spikes, generate Image from ImageMap.
            if (state->spikeCounter >= state->numSpikes) {

                printf("generating image with numspikes %d\n", state->numSpikes);
                haveimg[0] = true; // we got an image to classify

                //Code snippet to test normalization with fixed input
                /*for (int col_idx = 0; col_idx < state->sizeX; col_idx++) {
                 for (int row_idx = 0; row_idx < state->sizeY; row_idx++) {
                 state->ImageMap[col_idx][row_idx] = 256;
                 }
                 }*/
                /*
                 FILE*fp = fopen("test.txt","r");

                 if (fp == NULL) {

                 printf("Cant open the file\n");
                 exit(-1);

                 }
                 int col_idx, row_idx;
                 col_idx = 0;
                 row_idx = 0;
                 for (int read_idx = 0; read_idx < 64*64; read_idx++){
                 //  printf("Reading pixel %d\n", read_idx);
                 float pixel;
                 uint64_t scaled_pixel;
                 //   printf("1\n");
                 fscanf(fp, "%f\n", &pixel);
                 //    printf("2\n");
                 scaled_pixel = floor(pixel*256);
                 //    printf("3\n");
                 state->ImageMap[col_idx][row_idx] = scaled_pixel;
                 //    printf("4\n");
                 col_idx++;
                 if (col_idx == 64){
                 col_idx = 0;
                 row_idx++;
                 }

                 }
                 printf("File read done\n");
                 */
                if (!normalize_image_map_sigma(state, hist, CLASSIFY_IMG_SIZE)) {
                    caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
                            "Failed to normalize image map with 3 sigma range.");
                    return;
                };

                //reset values
                for (int col_idx = 0; col_idx < state->sizeX; col_idx++) {
                    for (int row_idx = 0; row_idx < state->sizeY; row_idx++) {
                        state->ImageMap[col_idx][row_idx] = 0;
                    }
                }
                state->spikeCounter = 0;

                // free chunks of memory
                state->frameRenderer = NULL;

            }
        } //CAER_POLARITY_ITERATOR_VALID_END
        free(onlyvalid);
    }/* **** SPIKE SECTION END *** */

}

static bool allocateImageMapSubsampled(imagegeneratorState state, int16_t sourceID, int16_t sizeX,
        int16_t sizeY) {
    // Get size information from source.
    sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
    if (sourceInfoNode == NULL) {
        // This should never happen, but we handle it gracefully.
        caerLog(CAER_LOG_ERROR, __func__, "Failed to get source info to allocate image map.");
        return (false);
    }

    // Initialize double-indirection contiguous 2D array, so that array[x][y]
    // is possible, see http://c-faq.com/aryptr/dynmuldimary.html for info.
    state->ImageMap = calloc((size_t) sizeX, sizeof(int64_t *));
    if (state->ImageMap == NULL) {
        return (false); // Failure.
    }

    state->ImageMap[0] = calloc((size_t) (sizeX * sizeY), sizeof(int64_t));
    if (state->ImageMap[0] == NULL) {
        free(state->ImageMap);
        state->ImageMap = NULL;

        return (false); // Failure.
    }

    for (size_t i = 1; i < (size_t) sizeX; i++) {
        state->ImageMap[i] = state->ImageMap[0] + (i * (size_t) sizeY);
    }

    // Assign max ranges for arrays (0 to MAX-1).
    state->sizeX = sizeX;
    state->sizeY = sizeY;

    // Init counters
    state->spikeCounter = 0;
    state->counterImg = 0;
    state->counterTxt = 0;
    state->counterFrame = 0;

    return (true);
}

