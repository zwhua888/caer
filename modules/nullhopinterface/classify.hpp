/* NullHop Interface for CNN in fpga
 *  Author: federico.corradi@inilabs.com
 */
#include <iostream>
#include <bitset>
#include <iomanip>
#include <stdio.h>
#include <termios.h>
#include <time.h>
#include <sys/mman.h>
#include <stdbool.h>
#include <math.h>
//#include <ncurses/curses.h>
#include <curses.h>
#include <cstddef>
#include <new>
#include <climits>
#include <sys/time.h>
#include <unistd.h>
#include <semaphore.h>
#include <signal.h>

#include <sched.h>
#include <thread>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <pthread.h>
#include "dirent.h"

#include "settings.h"

#define Assert(a,b) do { if (!(a)) { printf ( "Assertion failed, file %s, line %d\n", __FILE__, __LINE__); printf ( "Assertion: " #a "\n"); printf ( "ERR: " b "\n"); } } while (0)
#define NUM_MAC_BLOCKS 128
#define PATCH_SLACK 512
#define INPUT_STREAM_NUM_VALUES 2
#define MAX_CLUSTER_SIZE 8
#define MAX_NUM_INPUT_PIXELS 200000
#define PRE_TRUNCATION_TOLERANCE 10
#define MAX_WF_SIZE 1000000

#define MAX_BURST 64
#define MEM_SIZE 65535
#define MAX_S2MM_WAIT_COUNTER 10e4
#define MAX_LAYERS 100

#define safe_fscanf(a,b,c) do{ if(fscanf(a,b,c) == EOF) {fprintf(stderr,"EOF in fscanf at line %d\n",__LINE__); exit(1);} char comment[100];char * dummy = fgets(comment,100,a); } while(0)

#define IP1_IP_SIZE 784
#define IP1_OP_SIZE 128

#define IP2_IP_SIZE 128
#define IP2_OP_SIZE 2

#define device_addr_offset_ 0x40400000
#define dest_addr_offset_ 0x0F000000
#define src_addr_offset_ 0x0E000000

// Memory block AXI/DMA
#define MM2S_CONTROL_REGISTER_ 0x00
#define MM2S_STATUS_REGISTER_ 0x04
#define MM2S_START_ADDRESS_ 0x18
#define MM2S_LENGTH_ 0x28

#define S2MM_CONTROL_REGISTER_ 0x30
#define S2MM_STATUS_REGISTER_ 0x34
#define S2MM_DESTINATION_ADDRESS_ 0x48
#define S2MM_LENGTH_ 0x58

#define TRANSLEN_ 8 //bytes multiple of 4
#define BURST_ 10

#define FINISHED 3 // at the end of the processing

inline int truncateInt(int num, int numBits) {
	int result;
	result =
			(num < (-1 * (1 << (numBits - 1))) ? -1 * (1 << (numBits - 1)) : num);
	result =
			(result >= (1 << (numBits - 1)) ?
					((1 << (numBits - 1)) - 1) : result);
	return (result);
}

inline int min(unsigned int a, unsigned int b) {
	return (a < b ? a : b);
}

inline int max(unsigned int a, unsigned int b) {
	return (a > b ? a : b);
}

inline double fixedPointToDouble(int num, unsigned int numFracBits,
		int totalNumBits = -1) {
	if (totalNumBits > -1) {
		Assert(totalNumBits >= (int )numFracBits,
				"invalid arguments to fixedPointToDouble");
		num &= ~(~0 << totalNumBits);
	}
	return (num * 1.0 / (1 << numFracBits));
}

//reverse order compared to systemVerilog Struct
typedef struct {
	int s_output_pixel_stream_enable;
	int s_input_bus_valid;
	int s_input_bus_config_reg_addr[INPUT_STREAM_NUM_VALUES];
	int s_input_bus_type;
	int s_input_bus_data[INPUT_STREAM_NUM_VALUES];
	int s_resetn;
} t_input_sigs;

typedef struct {
	// signals added to multiLayer version
	int mac_compute_debug_s[MAX_CLUSTER_SIZE];
	int row_start_to_decoders_debug_s[MAX_CLUSTER_SIZE];
	int macs_input_data_available_debug_s[MAX_CLUSTER_SIZE];
	int idp_macs_pixels_channel_debug_s[MAX_CLUSTER_SIZE];
	int idp_macs_pixels_column_debug_s[MAX_CLUSTER_SIZE];
	int idp_macs_pixels_row_debug_s[MAX_CLUSTER_SIZE];
	int idp_macs_pixels_buffer_debug_s[MAX_CLUSTER_SIZE];

	// signals from driver_monitor_FPGA version
	int mac_output_pixels_ready_s[NUM_MAC_BLOCKS];
	int mac_output_pixels_bottomleft_s[NUM_MAC_BLOCKS];
	int mac_output_pixels_topleft_s[NUM_MAC_BLOCKS];

	int s_output_pixel_stream_valid;
	int s_output_pixel_stream;
	int s_input_bus_enable;

} t_output_sigs;

void * readThreadRoutine(void * arg);


class zs_driverMonitor {

	unsigned int m_nFracBits;

	// added struct for parameters of each layer (wrt FPGA tb).
	typedef struct {
		unsigned int image_compression_enabled;
		unsigned int kernel_size;
		unsigned int num_input_channels;
		unsigned int num_input_column;
		unsigned int num_input_rows;
		unsigned int num_output_channels;
		unsigned int pooling_enabled;
		unsigned int relu_enabled;
		unsigned int padding;

		int ***image; //input image to the layer
		int ****kernels;
		int *biases;

	} t_layerParams;

	t_layerParams * m_layerParams;

	// signals from FPGA tb, some are added.
	int m_nchIn, m_nchOut, m_imageWidth, m_hk, m_hinMax, m_wk,
			m_currentPixelPosition, m_numPixelsReady, m_nchOut_pseudo,
			m_nchIn_pseudo, m_outputChannelOffset, m_currentInputPass,
			m_numInputPasses;
	int ***m_image;
	int ***m_outputImage;
	int ***m_processedPixels;
	//  int ****m_patch;
	int m_currentPatchIndex;
	int ****m_kernelArray;

	bool m_completedKernelWrite, m_completedImageWrite, m_completedBiasWrite,
			m_completedConfigWrite, m_gotAllPixels, m_wroteImageDone;
	unsigned int m_kernelWritePos, m_imageWritePos;
	int m_currentOutputChannel, m_currentOutputYPos, m_currentOutputXPos;
	int m_currentInputChannel, m_currentInputYPos, m_currentInputXPos,
			m_currentInputRowShift;

	int m_currentOutputCol[NUM_MAC_BLOCKS];
	int m_currentOutputRow[NUM_MAC_BLOCKS];
	int m_numReadyMACs;
	int m_output_buffer_top, m_output_buffer_bot;
	int m_ready_flag[8];
	int m_oldValuesTop[8];
	int m_oldValuesBot[8];
	//  int *m_layerFinishTimes;

	int m_dummyKernels;
	int m_channel_decode_jump_mask;
	int m_contiguous_kernels;

	int *m_anchorChannel, *m_anchorColumn, *m_anchorRow, *m_anchorShift;
	int ****m_activePatch;

	int *m_previousPixelIndex;
	unsigned int m_macs_per_channel, m_log2_macs_per_channel;
	unsigned int m_pixelArrayWritePos, m_nPixelsArray, m_biasWritePos;
	int m_pixelArray[16];

	int *m_biases;

	int *m_initConfig;bool m_poolingEnabled, m_reluEnabled, m_encodingEnabled;bool m_imageCompressionEnabled;
	int instruction[2];
	int old_row;
	double m_imageScale;

	int m_currentDecodeIndex, m_currentDecodeSM;
	int m_idp_pixels[MAX_NUM_INPUT_PIXELS];
	int m_idp_row[MAX_NUM_INPUT_PIXELS];
	int m_idp_column[MAX_NUM_INPUT_PIXELS];
	int m_idp_channel[MAX_NUM_INPUT_PIXELS];
	int m_idp_shift[MAX_NUM_INPUT_PIXELS];
	int m_idp_mac_index[MAX_NUM_INPUT_PIXELS];

	int m_mac_stripe_index[8];
	unsigned int m_idp_decodePosition[8];bool m_idp_positionConsideredOnce[8];

	bool m_firstAxiWrite;

	unsigned int m_idpBuffer_pos;

	//int dh;
	off_t offset_control, offset_read, offset_write;
	unsigned int current_control, current_read, current_write;
	int s2mm_wait_counter;

	unsigned int max_nchIn;
	unsigned int max_nchOut;
	unsigned int max_imageWidth;
	unsigned int max_hinMax;
	unsigned int max_wk;
	unsigned int max_hk;
	unsigned int m_currentInitStep;
	unsigned int m_numLayers;

	unsigned int error_counter;
	unsigned int error_on_SMs;

	int m_outputLayerPadding;
	int m_inputLayerPadding;
	bool m_sent_start_pulse;
	bool m_sent_image_ready;

	char m_baseFileName[100];

	unsigned int tempPos;bool bottomRow;
	unsigned int shift;
	unsigned int chIdx, xPos, yPos;
	int maxPixel;

	int m_ip1_params[IP1_IP_SIZE * IP1_OP_SIZE + IP1_OP_SIZE];
	int m_ip2_params[IP2_IP_SIZE * IP2_OP_SIZE + IP2_OP_SIZE];
	int m_fc1_output[IP1_OP_SIZE];
	int m_fc2_output[IP2_OP_SIZE];

	long time_for_eval_fc;
	long time_for_eval_layer[MAX_LAYERS];
	long time_for_all;

	// added struct (wrt FPGA tb).
	/*typedef struct {
		int wf[MAX_WF_SIZE];
	} t_waveform;*/

public:

	int dha;
	unsigned int* virtual_address_;
	unsigned int* virtual_destination_address_;
	unsigned int* virtual_source_address_;

	sem_t signal_done;

	struct timeval start, end;
	long mtime, secs, usecs;
	unsigned int counter_l;

	unsigned int m_currentLayer;
	bool m_activeProcessing;
	bool threadRunning;
	pthread_t m_readThread;


	unsigned long long n_clkCycles = 0;
	t_input_sigs inputSigsInternal;

	//t_waveform m_compute_wfs; // weird behaviour

	volatile t_output_sigs *output_sigs;
	volatile t_input_sigs *input_sigs;
	friend void * readThreadRoutine(void *);

	enum CONFIG_TYPE {
		config_image_compression_enabled,
		config_pre_sm_counter_max,
		config_kernel_size,
		config_num_input_channels,
		config_num_input_column,
		config_num_input_rows,
		config_num_output_channels,
		config_pooling_enabled,
		config_relu_enabled,
		config_contiguous_kernels,
		config_num_macs_per_channel,
		config_input_channel_decode_jump_mask,
		config_kernel_memory_write_complete_pulse,
		config_kernel_memory_resetn_pulse,
		config_input_image_done,
		config_start_process_pulse = 19,
		config_padding_set = 21,
		config_last = 22 //dummy
	};

	void loadFCParams();
	void load_single_FC_layer(const char *fileName, int * ip_params,
			unsigned int paramSize);
	void evaluateFCLayers();
	int ipow(int base, int exp);


	zs_driverMonitor() {
		m_currentLayer = 0;
		m_layerParams = NULL;
		m_currentInputPass = 0;

		mtime = 0;
		secs = 0 ;
		usecs = 0;
		time_for_eval_fc = -1;
		for (int i=0; i <= MAX_LAYERS ; i++){
			time_for_eval_layer[i] = -1;
		}
		time_for_all = -1;
		counter_l = 0;

		input_sigs = new t_input_sigs;
		output_sigs = new t_output_sigs;
		memset((void *)input_sigs, 0, sizeof(t_input_sigs));
		memset((void *) output_sigs, 0, sizeof(t_output_sigs));

		m_activeProcessing = false;
		m_nFracBits = 8;

		m_initConfig = new int[(int) (config_last)];

		dha = open("/dev/mem", O_RDWR | O_SYNC); // Open /dev/mem which represents the whole physical memory
		virtual_address_ = (unsigned int *) mmap(NULL, 65535,
		PROT_READ | PROT_WRITE, MAP_SHARED, dha, device_addr_offset_); // Memory map AXI Lite register block
		virtual_destination_address_ = (unsigned int *) mmap(NULL, 65535,
		PROT_READ | PROT_WRITE, MAP_SHARED, dha, dest_addr_offset_); // Memory map destination address
		virtual_source_address_ = (unsigned int *) mmap(NULL, 65535,
		PROT_READ | PROT_WRITE, MAP_SHARED, dha, src_addr_offset_); // Memory map source address

		initializeInternalVariables();
		readNetwork(NETWORK_FILE);

	}

	double getTimeForAll(){
		return(this->time_for_all);
	}
    void setTotalTime(double timer);
	int launchThread();
	void closeThread();
	int setCurrentLayer(unsigned int layerIndex);
	void readNetwork(const char *fileName);
	void checkMarker(FILE *fp, const char * marker);
	void readLayer(FILE *fp, t_layerParams &layerParam, bool firstLayer);
	void initializeInternalVariables();
	void readKernels(FILE *fp, t_layerParams &layerParam);
	void readBiases(FILE *fp, t_layerParams & layerParam);
	void readPixels(FILE *fp, t_layerParams & layerParam);
	void initializePixelArray(t_layerParams & layerParam);
	void initializeBiasArray(t_layerParams & layerParam);
	void initializeKernelArray(t_layerParams & layerParam);
	void initializeConfigArray();

	void dma_set_(unsigned int* dma_virtual_address, int offset,
			unsigned int value);
	unsigned int dma_get_(unsigned int* dma_virtual_address, int offset);
	void resetAxiBus();
	void waitValidAxiDataToRead_(int wordsNumber);
	void initNet();
	void setInternalVariables();

	void dma_mm2s_sync_(unsigned int* dma_virtual_address);
	void dma_s2mm_sync_(unsigned int* dma_virtual_address);
	int dma_s2mm_status_(unsigned int* dma_virtual_address);
	int dma_mm2s_status_(unsigned int* dma_virtual_address);
	void memdump_(char* virtual_address, int byte_count);
	void hexDump_(char *desc, void *addr, int len);

	int writeAxiCommit_(int wordsNumber, unsigned int startPos);
	void dma_s2mm_sync_halted_and_notIDLE_(unsigned int* dma_virtual_address);
	void stopS2MM_(void);
	void memdump_checking_(char* virtual_address, int byte_count);

	~zs_driverMonitor() { }

	int getGroundTruthPixel(unsigned int outputChannel, int xPos, int yPos,
			int & fullResResult, bool debug = false);
	void sendConfigData(CONFIG_TYPE config_type, int data);bool initializationLoop();
	void writeBiasValue(int biasValue, int biasAddress);
	void writeKernelValue(int kernelValue[2], int validMask);
	void writePixels(int pos0, int pos1, bool pos1_valid, int instruction[2]);
	uint16_t int_to_short(int data);
	void writeToAxi();
	void axiWriteCommit();
	unsigned int reverseInt(unsigned int src);
	void readFromAxi();
	bool matchToPatch(int output_pixel, int pixel_ch,
			int pixel_xpos, int pixel_ypos, unsigned int mac_index);
	void phase2_step();
	void pixel_step();
	void dumpImage();
	double getSparsity();
	int processingLoop(unsigned int currentStep);
	int runLoop(void);

	int file_set(uint8_t * picture);

	void printStatus();
	int loadImage();
	int threadExists();
	void sw_reset();

private:
	char * file_i;

};


