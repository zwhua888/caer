/* NullHop Interface for CNN acceleration
 *  Author: federico.corradi@inilabs.com
 */
#include "classify.hpp"
#include <fstream>
#include <iostream>

int zs_driverMonitor::file_set(uint16_t * picture) {

	//std::ofstream image_file;
	//image_file.open("input_images.txt", std::ios::app);
	//image_file << "#INPUT IMAGE\n";
	int counter = 0;
	int tmp_img = 0;
	for (unsigned int i = 0; i < 1; ++i) {
		for (unsigned int j = 0; j < 36; ++j) {
			for (unsigned int k = 0; k < 36; ++k) {
				tmp_img = picture[counter] * 256;
				if (tmp_img > 32767 || tmp_img < -32768) {
					if (tmp_img > 32767) {
						this->m_layerParams[0].image[i][j][k] = (int) 32767*0.00390625;
					} else {
						this->m_layerParams[0].image[i][j][k] =(int) -32768*0.00390625;
					}
				} else {
					this->m_layerParams[0].image[i][j][k] = (int) tmp_img*0.00390625;
				}
				counter += 1;
			}
		}
	}
	m_image = this->m_layerParams[0].image;

	this->n_clkCycles = 0;

	while (this->processingLoop(this->n_clkCycles) == FINISHED) {

		this->n_clkCycles = 0;
		this->m_activeProcessing = false;
		this->m_currentLayer = 0;

	}

	this->printStatus();

	return (0);
}

void zs_driverMonitor::setTotalTime(double timer) {
	this->time_for_all = timer;
}

void zs_driverMonitor::initNet() {

	resetAxiBus();
	loadFCParams(); // load fully connected weights

}

void zs_driverMonitor::dma_set_(unsigned int* dma_virtual_address, int offset,
		unsigned int value) {
	dma_virtual_address[offset >> 2] = value;
}

unsigned int zs_driverMonitor::dma_get_(unsigned int* dma_virtual_address,
		int offset) {
	return (dma_virtual_address[offset >> 2]);
}

void zs_driverMonitor::resetAxiBus() {
	dma_set_(virtual_address_, S2MM_CONTROL_REGISTER_, 4);
	dma_set_(virtual_address_, MM2S_CONTROL_REGISTER_, 4);
	dma_set_(virtual_address_, S2MM_CONTROL_REGISTER_, 0);
	dma_set_(virtual_address_, MM2S_CONTROL_REGISTER_, 0);
}

void zs_driverMonitor::waitValidAxiDataToRead_(int wordsNumber) {

	dma_set_(virtual_address_, S2MM_STATUS_REGISTER_, 2); // Clear idle
	dma_set_(virtual_address_, S2MM_STATUS_REGISTER_, 0x1000); // Clear IOC_Irq
	dma_set_(virtual_address_, S2MM_DESTINATION_ADDRESS_, dest_addr_offset_);
	dma_set_(virtual_address_, S2MM_CONTROL_REGISTER_, 0x0001); //0xf001
	dma_set_(virtual_address_, S2MM_LENGTH_, TRANSLEN_ * wordsNumber);
	dma_s2mm_sync_(virtual_address_); // Wait until Idle or IOC_Irq bit is 1

}

void zs_driverMonitor::dma_mm2s_sync_(unsigned int* dma_virtual_address) {
	unsigned int mm2s_status = dma_get_(dma_virtual_address,
	MM2S_STATUS_REGISTER_);

	while (!(mm2s_status & 1 << 12) || !(mm2s_status & 1 << 1)) {
		mm2s_status = dma_get_(dma_virtual_address, MM2S_STATUS_REGISTER_);
	}
}

void zs_driverMonitor::dma_s2mm_sync_(unsigned int* dma_virtual_address) {
	unsigned int s2mm_status = dma_get_(dma_virtual_address,
	S2MM_STATUS_REGISTER_);

	while (!(s2mm_status & (1 << 12)) || !(s2mm_status & (1 << 1))) {
		s2mm_status = dma_get_(dma_virtual_address, S2MM_STATUS_REGISTER_);

	}
}

void zs_driverMonitor::dma_s2mm_sync_halted_and_notIDLE_(
		unsigned int* dma_virtual_address) {
	unsigned int s2mm_status = dma_get_(dma_virtual_address,
	S2MM_STATUS_REGISTER_);

	while ((s2mm_status & (1 << 12)) || (s2mm_status & (1 << 1))) {
		s2mm_status = dma_get_(dma_virtual_address, S2MM_STATUS_REGISTER_);
	}
}

int zs_driverMonitor::dma_s2mm_status_(unsigned int* dma_virtual_address) {
	unsigned int status = dma_get_(dma_virtual_address, S2MM_STATUS_REGISTER_);
	printf("Stream to memory-mapped status (0x%08x@0x%02x):", status,
	S2MM_STATUS_REGISTER_);
	if (status & 0x00000001) {
		printf(" halted\n");
		return (0);
	} else {
		printf(" running\n");
		return (1);
	}
	if (status & 0x00000002) {
		printf(" idle\n");
		return (1);
	}
	if (status & 0x00000008) {
		printf(" SGIncld\n");
		return (1);
	}
	if (status & 0x00000010) {
		printf(" DMAIntErr\n");
		return (1);
	}
	if (status & 0x00000020) {
		printf(" DMASlvErr\n");
		return (1);
	}
	if (status & 0x00000040) {
		printf(" DMADecErr\n");
		return (1);
	}
	if (status & 0x00000100) {
		printf(" SGIntErr\n");
		return (1);
	}
	if (status & 0x00000200) {
		printf(" SGSlvErr\n");
		return (1);
	}
	if (status & 0x00000400) {
		printf(" SGDecErr\n");
		return (1);
	}
	if (status & 0x00001000) {
		printf(" IOC_Irq\n");
		return (1);
	}
	if (status & 0x00002000) {
		printf(" Dly_Irq\n");
		return (1);
	}
	if (status & 0x00004000) {
		printf(" Err_Irq\n");
		return (1);
	}
}

int zs_driverMonitor::dma_mm2s_status_(unsigned int* dma_virtual_address) {
	unsigned int status = dma_get_(dma_virtual_address, MM2S_STATUS_REGISTER_);
	printf("Memory-mapped to stream status (0x%08x@0x%02x):", status,
	MM2S_STATUS_REGISTER_);
	if (status & 0x00000001) {
		printf(" halted\n");
		return (0);
	} else {
		printf(" running\n");
		return (1);
	}
	if (status & 0x00000002) {
		printf(" idle\n");
		return (1);
	}
	if (status & 0x00000008) {
		printf(" SGIncld\n");
		return (1);
	}
	if (status & 0x00000010) {
		printf(" DMAIntErr\n");
		return (1);
	}
	if (status & 0x00000020) {
		printf(" DMASlvErr\n");
		return (1);
	}
	if (status & 0x00000040) {
		printf(" DMADecErr\n");
		return (1);
	}
	if (status & 0x00000100) {
		printf(" SGIntErr\n");
		return (1);
	}
	if (status & 0x00000200) {
		printf(" SGSlvErr\n");
		return (1);
	}
	if (status & 0x00000400) {
		printf(" SGDecErr\n");
		return (1);
	}
	if (status & 0x00001000) {
		printf(" IOC_Irq\n");
		return (1);
	}
	if (status & 0x00002000) {
		printf(" Dly_Irq\n");
		return (1);
	}
	if (status & 0x00004000) {
		printf(" Err_Irq\n");
		return (1);
	}
}

void zs_driverMonitor::memdump_(char* virtual_address, int byte_count) {
	char *p = virtual_address;
	int offset;
	unsigned int data = 0;
	unsigned int data_low = 0;

	for (offset = 0; offset < byte_count; offset++) {
		data |= (p[offset] & 0xFF) << ((offset % 4) * 8);
		if (offset % 8 == 7) {
			printf("0x%08x%08x\n", data, data_low);
			data = 0;
			data_low = 0;
		} else {
			if (offset % 4 == 3) {
				data_low = data;
				data = 0;
			}
		}
	}
}

void zs_driverMonitor::memdump_checking_(char* virtual_address,
		int byte_count) {
	char *p = virtual_address;
	int offset;
	unsigned int data = 0;
	unsigned int data_low = 0;
	unsigned int data_low_bkp = 0;

	for (offset = 0; offset < byte_count; offset++) {
		data |= (p[offset] & 0xFF) << ((offset % 4) * 8);
		if (offset % 8 == 7) {
			printf("0x%08x%08x\n", data, data_low);
			if (data != 3 || data_low != data_low_bkp) {
				printf(
						"Error in the sequence. is expected: high --> 00000003, low --> %d. Received: high --> %d, low --> %d",
						data_low_bkp, data, data_low);
			} else if (data_low == 0x3f) {
				data_low_bkp = 0;
			} else {
				data_low_bkp += 0x100;
			}
			data = 0;
			data_low = 0;
		} else {
			if (offset % 4 == 3) {
				data_low = data;
				data = 0;
			}
		}
	}
}

int zs_driverMonitor::launchThread() {

	// high priority thread
	pthread_attr_t attr;
	struct sched_param param;

	pthread_attr_init(&attr);
	pthread_attr_getschedparam(&attr, &param);
	(param.sched_priority)++;
	pthread_attr_setschedparam(&attr, &param);

	int err = -1;
	int counter = 0;
	while (err != 0) {
		err = pthread_create(&m_readThread, &attr, readThreadRoutine,
				(void *) this);
		if (err != 0) {
			printf("+++++ ERROR : while creating read thread.. exiting %d\n",
					err);
		}
		counter += 1;
		if (counter > 10) {
			return (-1);
		}
	}
	this->threadRunning = true;
	return (0);

}

int zs_driverMonitor::loadImage() {

	return (1);
}
void zs_driverMonitor::closeThread() {
	pthread_join(m_readThread, NULL);
}

void zs_driverMonitor::loadFCParams() {
	load_single_FC_layer(IP1_PARAMS_FILE, m_ip1_params,
			sizeof(m_ip1_params) / sizeof(m_ip1_params[0]));
	load_single_FC_layer(IP2_PARAMS_FILE, m_ip2_params,
			sizeof(m_ip2_params) / sizeof(m_ip2_params[0]));

}

void zs_driverMonitor::load_single_FC_layer(const char *fileName,
		int * ip_params, unsigned int paramSize) {
	FILE *fp = fopen(fileName, "r");
	for (unsigned int i = 0; i < paramSize; ++i)
		safe_fscanf(fp, "%d", &ip_params[i]);
	fclose(fp);
}

void zs_driverMonitor::evaluateFCLayers() {
	unsigned int idx_tot_fc1 = 0;
	int max_height = (m_hinMax - m_hk + 1);

	if (m_poolingEnabled) {
		max_height /= 2;
	}
	int max_width = (m_imageWidth - m_wk + 1);
	if (m_poolingEnabled) {
		max_width /= 2;
	}

	/*int m_nchOuta = m_nchOut;
	 int max_widtha = max_width;
	 int max_heighta = max_height;*/
	for (int i = 0; i < IP1_OP_SIZE; ++i) {
		m_fc1_output[i] = 0;
		for (int j = 0; j < m_nchOut; ++j) { //m_nchIn; ++j) { // m_nchOut
			for (int k = 0; k < max_width; ++k) { //m_hinMax; ++k) { //max_width
				for (int l = 0; l < max_height; ++l) { //m_imageWidth //max_height
					m_fc1_output[i] += m_ip1_params[idx_tot_fc1]
							* m_image[j][k][l];
					//printf("m_nchIn %d m_hinMax %d m_imageWidth %d, m_poolingEnabled %d \n", m_nchOut, max_width, max_height, m_poolingEnabled);
					//printf(" m_ip1_params[%d] %d\n",idx_tot_fc1, m_ip1_params[idx_tot_fc1]);
					idx_tot_fc1++;
				}

			}
		}
		m_fc1_output[i] = (m_fc1_output[i] / 256)
				+ m_ip1_params[m_nchIn * m_hinMax * m_imageWidth * IP1_OP_SIZE
						+ i];
		m_fc1_output[i] = m_fc1_output[i] > 0 ? m_fc1_output[i] : 0;
		if (i == IP1_OP_SIZE - 1) {
			fprintf(stderr, "m_fc1_output[%d] %d\n", i, m_fc1_output[i]);
		}
	}

	for (int i = 0; i < IP2_OP_SIZE; ++i) {
		m_fc2_output[i] = 0;
		for (int j = 0; j < IP1_OP_SIZE; ++j) {
			m_fc2_output[i] += m_ip2_params[i * IP1_OP_SIZE + j]
					* m_fc1_output[j];
			//printf("m_ip2_params[%d * %d + %d] %d\n", i, IP1_OP_SIZE, j,  m_ip2_params[i * IP1_OP_SIZE + j]);
		}

		m_fc2_output[i] = (m_fc2_output[i] / 256)
				+ m_ip2_params[IP2_OP_SIZE * IP1_OP_SIZE + i];

		fprintf(stderr, " m_fc2_output[%d] %d\n", i, m_fc2_output[i]);
	}

	//for face net only
	if (m_fc2_output[1] > m_fc2_output[0]) {
		fprintf(stderr, "\nFACE DETECTED\n");
		system("echo 1 >> /dev/ttyUSB0");
	}

}

void zs_driverMonitor::sw_reset() {
	//resetAxiBus();
	virtual_source_address_[0] = 0;
	virtual_source_address_[1] = 0x80000000;
	virtual_source_address_[2] = 0;
	virtual_source_address_[3] = 0;
	writeAxiCommit_(2, 0);
}

int zs_driverMonitor::ipow(int base, int exp) {
	int result = 1;
	while (exp) {
		if (exp & 1)
			result *= base;
		exp >>= 1;
		base *= base;
	}
	return (result);
}

int zs_driverMonitor::setCurrentLayer(unsigned int layerIndex) {
	if (layerIndex >= m_numLayers) {
		return (FINISHED);
	}
	m_imageWidth = m_layerParams[layerIndex].num_input_column;
	m_hinMax = m_layerParams[layerIndex].num_input_rows;
	m_nchIn = m_layerParams[layerIndex].num_input_channels;
	m_nchOut = m_nchOut_pseudo = m_layerParams[layerIndex].num_output_channels;
	m_hk = m_wk = m_layerParams[layerIndex].kernel_size;
	m_imageCompressionEnabled =
			m_layerParams[layerIndex].image_compression_enabled;
	m_reluEnabled = m_layerParams[layerIndex].relu_enabled;
	m_poolingEnabled = m_layerParams[layerIndex].pooling_enabled;
	m_encodingEnabled = m_reluEnabled;

	int numKernelsPerChannel = m_hk * m_wk * m_nchIn;
	int requiredMacsPerChannel_memory = numKernelsPerChannel / (4096) + 1;
	int requiredMacsPerChannel_numChannels;
	if (m_nchOut > 128) {
		int outputChannelBlocks = m_nchOut / 128;
		int outputChannelsPerBlock = m_nchOut
				/ (outputChannelBlocks + (m_nchOut % 128 ? 1 : 0));
		requiredMacsPerChannel_numChannels = 128 / outputChannelsPerBlock;
	} else
		requiredMacsPerChannel_numChannels = 128 / m_nchOut;

	int requiredMacsPerChannel = (
			requiredMacsPerChannel_numChannels > requiredMacsPerChannel_memory ?
					requiredMacsPerChannel_numChannels :
					requiredMacsPerChannel_memory);
	if (requiredMacsPerChannel == 1)
		requiredMacsPerChannel = 1;
	else if (requiredMacsPerChannel <= 2)
		requiredMacsPerChannel = 2;
	else if (requiredMacsPerChannel <= 4)
		requiredMacsPerChannel = 4;
	else if (requiredMacsPerChannel <= 8)
		requiredMacsPerChannel = 8;
	else {
		fprintf(stderr, "invalid macs per channel %d\n",
				requiredMacsPerChannel);
	}

	m_nchOut = m_nchOut_pseudo = 128 / requiredMacsPerChannel;
	m_numInputPasses = m_layerParams[layerIndex].num_output_channels / m_nchOut;
	m_outputChannelOffset = m_currentInputPass * m_nchOut;

	int nearestPow2 = 1;
	while (nearestPow2 < m_nchIn) {
		nearestPow2 *= 2;
	}

	if (requiredMacsPerChannel == 1) {
		m_dummyKernels = 0;
		m_contiguous_kernels = m_nchIn * m_wk * m_hk;
		m_nchIn_pseudo = m_nchIn;
		m_channel_decode_jump_mask = nearestPow2 - 1;
	} else {
		m_dummyKernels = (nearestPow2 - m_nchIn) * m_wk * m_hk;

		if (nearestPow2 <= requiredMacsPerChannel) {
			m_contiguous_kernels = m_wk * m_hk;
			m_dummyKernels += (requiredMacsPerChannel - nearestPow2) * m_wk
					* m_hk;
		} else {
			m_contiguous_kernels = (nearestPow2 / requiredMacsPerChannel) * m_wk
					* m_hk;
		}

		m_nchIn_pseudo = m_nchIn + (m_dummyKernels / (m_wk * m_hk));

		m_channel_decode_jump_mask = (m_contiguous_kernels / (m_wk * m_hk)) - 1;
	}

	m_macs_per_channel = requiredMacsPerChannel;

	m_sent_start_pulse = false;
	m_sent_image_ready = false;
	m_kernelArray = m_layerParams[layerIndex].kernels;
	m_biases = m_layerParams[layerIndex].biases;

	m_image = m_layerParams[layerIndex].image;
	m_outputImage = m_layerParams[layerIndex + 1].image;

	m_idpBuffer_pos = 0;

	m_inputLayerPadding = m_layerParams[layerIndex].padding;
	m_outputLayerPadding = m_layerParams[layerIndex + 1].padding;

	return (0);

}

void zs_driverMonitor::readNetwork(const char *fileName) {
	FILE *fp = fopen(fileName, "r");
	if (!fp) {
		fprintf(stderr, "can not open network file");
		exit(1);
	}
	safe_fscanf(fp, "%d", &m_numLayers);
	if (m_layerParams) {
		fprintf(stderr, "readNetwork can only be called once\n");
		exit(1);
	}
	m_layerParams = new t_layerParams[m_numLayers + 1];

	for (unsigned int i = 0; i < m_numLayers; i++) {
		readLayer(fp, m_layerParams[i], i == 0);
	}

	//initialize the final layer (not really a layer, just a storage for the top layer output)
	int divisor = (m_layerParams[m_numLayers - 1].pooling_enabled ? 2 : 1);
	int last_kernel_size = m_layerParams[m_numLayers - 1].kernel_size;
	memset(&m_layerParams[m_numLayers], 0, sizeof(t_layerParams));

	m_layerParams[m_numLayers].num_input_channels = m_layerParams[m_numLayers
			- 1].num_output_channels;
	m_layerParams[m_numLayers].num_input_column =
			(m_layerParams[m_numLayers - 1].num_input_column - last_kernel_size
					+ 1 + 2 * m_layerParams[m_numLayers - 1].padding) / divisor;
	m_layerParams[m_numLayers].num_input_rows =
			(m_layerParams[m_numLayers - 1].num_input_rows - last_kernel_size
					+ 1 + 2 * m_layerParams[m_numLayers - 1].padding) / divisor;
	initializePixelArray(m_layerParams[m_numLayers]);
	initializeKernelArray(m_layerParams[m_numLayers]);
	initializeBiasArray(m_layerParams[m_numLayers]);

	error_counter = 0;
	error_on_SMs = 0;
}

void zs_driverMonitor::checkMarker(FILE *fp, const char * marker) {
	char line[256];
	safe_fscanf(fp, "%s", line);
	if (!strstr(line, marker)) {
		fprintf(stderr, "failed to find marker %s\n", marker);
		exit(1);
	}
}

void zs_driverMonitor::readLayer(FILE *fp, t_layerParams &layerParam,
bool firstLayer) {
	unsigned int * paramsAsArray = ((unsigned int *) (&layerParam));

	unsigned int numParams = 9;

	for (unsigned int i = 0; i < numParams; ++i) {
		unsigned int temp;
		safe_fscanf(fp, "%u", &temp);
		paramsAsArray[i] = temp;
	}

	initializeKernelArray(layerParam);
	initializeBiasArray(layerParam);
	initializePixelArray(layerParam);

	checkMarker(fp, "#KERNELS#");
	readKernels(fp, layerParam);

	checkMarker(fp, "#BIASES#");
	readBiases(fp, layerParam);

	if (firstLayer) {
		checkMarker(fp, "#PIXELS#");
		readPixels(fp, layerParam);
	}
}

void zs_driverMonitor::setInternalVariables() {

	current_control = current_write = current_read = 0;
	m_currentInitStep = 0;

	m_pixelArrayWritePos = m_nPixelsArray = 0;

	memset(m_oldValuesTop, 0, sizeof(int) * 8);
	memset(m_oldValuesBot, 0, sizeof(int) * 8);
	memset(m_idp_decodePosition, 0, sizeof(unsigned int) * 8);
	memset(m_mac_stripe_index, 0, sizeof(unsigned int) * 8);

	for (unsigned int i = 0; i < 8; ++i)
		m_idp_positionConsideredOnce[i] = false;

	m_log2_macs_per_channel = 0;
	unsigned int temp = m_macs_per_channel;
	while (temp >>= 1)
		m_log2_macs_per_channel++;

	m_currentDecodeIndex = -16;
	m_currentDecodeSM = 0;

	m_firstAxiWrite = true;

	m_currentPixelPosition = m_numPixelsReady = 0;
	m_idpBuffer_pos = 0;

	m_kernelWritePos = 0;
	m_imageWritePos = 0;
	m_currentOutputChannel = 0;
	m_currentOutputYPos = 0;
	m_currentOutputXPos = 0;
	m_currentInputChannel = 0;
	m_currentInputYPos = 0;
	m_currentInputXPos = 0;
	m_wroteImageDone = 0;
	m_currentInputRowShift = 0;
	m_biasWritePos = 0;

	m_completedKernelWrite = false;
	m_completedImageWrite = false;
	m_completedBiasWrite = false;
	m_gotAllPixels = false;
	m_completedConfigWrite = false;

	for (unsigned int i = 0; i < NUM_MAC_BLOCKS; ++i) {
		m_currentOutputCol[i] = 0;
		m_currentOutputRow[i] = 0;
	}

	return;
}

void zs_driverMonitor::initializeInternalVariables() {

	current_control = current_write = current_read = 0;
	m_currentInitStep = 0;

	m_pixelArrayWritePos = m_nPixelsArray = 0;

	memset(m_oldValuesTop, 0, sizeof(int) * 8);
	memset(m_oldValuesBot, 0, sizeof(int) * 8);
	memset(m_idp_decodePosition, 0, sizeof(unsigned int) * 8);
	memset(m_mac_stripe_index, 0, sizeof(unsigned int) * 8);

	for (unsigned int i = 0; i < 8; ++i)
		m_idp_positionConsideredOnce[i] = false;

	m_log2_macs_per_channel = 0;
	unsigned int temp = m_macs_per_channel;
	while (temp >>= 1)
		m_log2_macs_per_channel++;

	m_currentDecodeIndex = -16;
	m_currentDecodeSM = 0;

	m_firstAxiWrite = true;

	m_currentPixelPosition = m_numPixelsReady = 0;
	m_idpBuffer_pos = 0;

	m_kernelWritePos = 0;
	m_imageWritePos = 0;
	m_currentOutputChannel = 0;
	m_currentOutputYPos = 0;
	m_currentOutputXPos = 0;
	m_currentInputChannel = 0;
	m_currentInputYPos = 0;
	m_currentInputXPos = 0;
	m_wroteImageDone = 0;
	m_currentInputRowShift = 0;
	m_biasWritePos = 0;

	m_completedKernelWrite = false;
	m_completedImageWrite = false;
	m_completedBiasWrite = false;
	m_gotAllPixels = false;
	m_completedConfigWrite = false;

	for (unsigned int i = 0; i < NUM_MAC_BLOCKS; ++i) {
		m_currentOutputCol[i] = 0;
		m_currentOutputRow[i] = 0;
	}

	try {
		m_anchorChannel = new int[m_macs_per_channel];
		m_anchorColumn = new int[m_macs_per_channel];
		m_anchorRow = new int[m_macs_per_channel];
		m_anchorShift = new int[m_macs_per_channel];
		m_previousPixelIndex = new int[m_macs_per_channel];
		m_activePatch = new int ***[m_macs_per_channel];

	} catch (const std::bad_array_new_length &e) {
		std::cout << e.what() << '\n';
	}

	for (unsigned int p = 0; p < m_macs_per_channel; ++p) {
		m_anchorChannel[p] = m_anchorColumn[p] = m_anchorRow[p] =
				m_anchorShift[p] = 0;
		m_previousPixelIndex[p] = -1;
		m_activePatch[p] = new int**[m_nchOut];

		for (int i = 0; i < m_nchOut; ++i) {
			m_activePatch[p][i] = new int *[m_hinMax];

			for (int j = 0; j < m_hinMax; ++j) {
				m_activePatch[p][i][j] = new int[m_imageWidth];

				for (int k = 0; k < m_imageWidth; ++k) {

					if (p == 0)
						m_activePatch[p][i][j][k] = m_biases[i] * 256;
					else
						m_activePatch[p][i][j][k] = 0;
				}

			}
		}
	}

	m_processedPixels = new int **[m_nchIn];

	for (int i = 0; i < m_nchIn; ++i) {
		m_processedPixels[i] = new int *[m_hinMax + m_inputLayerPadding * 2];
		for (int j = 0; j < m_hinMax + m_inputLayerPadding * 2; ++j) {
			m_processedPixels[i][j] = new int[m_imageWidth
					+ m_inputLayerPadding * 2];
			for (int k = 0; k < m_imageWidth + m_inputLayerPadding * 2; ++k) {
				m_processedPixels[i][j][k] = 0;

			}

		}

	}

}

void zs_driverMonitor::readKernels(FILE *fp, t_layerParams &layerParam) {
	int **** &kernelArray = layerParam.kernels;
	unsigned int kernel_size = layerParam.kernel_size;
	unsigned int num_input_channels = layerParam.num_input_channels;
	unsigned int num_output_channels = layerParam.num_output_channels;

	for (unsigned int i = 0; i < num_output_channels; ++i) {
		for (unsigned int j = 0; j < num_input_channels; ++j) {
			for (unsigned int k = 0; k < kernel_size; ++k) {
				for (unsigned int l = 0; l < kernel_size; ++l) {
					safe_fscanf(fp, "%d", &kernelArray[i][j][k][l]);
				}

			}
		}
	}
}

void zs_driverMonitor::readBiases(FILE *fp, t_layerParams & layerParam) {
	int * &biases = layerParam.biases;
	unsigned int num_output_channels = layerParam.num_output_channels;

	for (unsigned int i = 0; i < num_output_channels; ++i) {
		safe_fscanf(fp, "%d", &biases[i]);
	}

}

void zs_driverMonitor::readPixels(FILE *fp, t_layerParams & layerParam) {
	int *** &image = layerParam.image;
	unsigned int num_channels = layerParam.num_input_channels;
	unsigned int height = layerParam.num_input_rows;
	unsigned int width = layerParam.num_input_column;
	unsigned int padding = 0;    //layerParam.padding;

	for (unsigned int i = 0; i < num_channels; ++i)
		for (unsigned int j = 0; j < height - padding * 2; ++j)
			for (unsigned int k = 0; k < width - padding * 2; ++k) {
				safe_fscanf(fp, "%i", &image[i][j + padding][k + padding]);
			}
}

void zs_driverMonitor::initializePixelArray(t_layerParams & layerParam) {
	int *** &image = layerParam.image;
	unsigned int num_channels = layerParam.num_input_channels;
	unsigned int height = layerParam.num_input_rows;
	unsigned int width = layerParam.num_input_column;

	image = new int **[num_channels];
	for (unsigned int i = 0; i < num_channels; ++i) {
		image[i] = new int *[height];
		for (unsigned int j = 0; j < height; ++j) {
			image[i][j] = new int[width];
			for (unsigned int k = 0; k < width; ++k) {
				image[i][j][k] = 0;
			}

		}
	}
}

void zs_driverMonitor::initializeBiasArray(t_layerParams & layerParam) {
	int * &biases = layerParam.biases;
	unsigned int num_output_channels = layerParam.num_output_channels;

	try {
		biases = new int[num_output_channels];
	} catch (const std::bad_array_new_length &e) {
		std::cout << e.what() << '\n';
	}

	for (unsigned int i = 0; i < num_output_channels; ++i)
		biases[i] = 0;
}

void zs_driverMonitor::initializeKernelArray(t_layerParams & layerParam) {
	int **** &kernelArray = layerParam.kernels;
	unsigned int kernel_size = layerParam.kernel_size;
	unsigned int num_input_channels = layerParam.num_input_channels;
	unsigned int num_output_channels = layerParam.num_output_channels;

	try {
		kernelArray = new int ***[num_output_channels];
	} catch (const std::bad_array_new_length &e) {
		std::cout << e.what() << '\n';
	}

	for (unsigned int i = 0; i < num_output_channels; ++i) {
		kernelArray[i] = new int **[num_input_channels];
		for (unsigned int j = 0; j < num_input_channels; ++j) {
			kernelArray[i][j] = new int*[kernel_size];
			for (unsigned int k = 0; k < kernel_size; ++k) {
				kernelArray[i][j][k] = new int[kernel_size];
				for (unsigned int l = 0; l < kernel_size; ++l) {
					kernelArray[i][j][k][l] = 0;
				}

			}
		}
	}
}

void zs_driverMonitor::initializeConfigArray() {
	m_initConfig[0] = m_imageCompressionEnabled; //config_image_compression_enabled,
	m_initConfig[1] = (128 / (16 * m_macs_per_channel)) - 1; //config_pre_sm_counter_max,
	m_initConfig[2] = m_wk; //config_kernel_size,
	m_initConfig[3] = m_nchIn; //config_num_input_channels,
	m_initConfig[4] = m_imageWidth; //config_num_input_column,
	m_initConfig[5] = m_hinMax; //config_num_input_rows,
	m_initConfig[6] = m_nchOut; //config_num_output_channels,
	m_initConfig[7] = m_poolingEnabled; //config_pooling_enabled,
	m_initConfig[8] = m_reluEnabled; //config_relu_enabled,
	m_initConfig[9] = m_contiguous_kernels; //config_contiguous_kernels
	m_initConfig[10] = m_macs_per_channel - 1; //config_num_macs_per_channel,
	m_initConfig[11] = m_channel_decode_jump_mask; //config_input_channel_decode_jump_mask,
	m_initConfig[12] = 0; // config_kernel_memory_write_complete_pulse,
	m_initConfig[13] = 0; //config_kernel_memory_resetn_pulse
	m_initConfig[14] = 0; //config_input_image_done

}

void zs_driverMonitor::sendConfigData(CONFIG_TYPE config_type, int data) {
	input_sigs->s_input_bus_valid = 1;
	input_sigs->s_input_bus_config_reg_addr[0] = ((int) config_type);
	input_sigs->s_input_bus_data[0] = data;
	input_sigs->s_input_bus_type = 3;
	writeToAxi();
}

bool zs_driverMonitor::initializationLoop() {

	if (m_currentInitStep <= config_kernel_memory_resetn_pulse) {
		sendConfigData((CONFIG_TYPE) m_currentInitStep,
				m_initConfig[m_currentInitStep]);
		//fprintf(stderr, "initi step %d\n", m_currentInitStep);
		++m_currentInitStep;
		return (false);
	} else {
		if (m_currentInitStep == config_kernel_memory_resetn_pulse + 1) {
			sendConfigData(config_padding_set, m_inputLayerPadding);
			++m_currentInitStep;
			return (false);
		}
		return (true);
	}

}

void zs_driverMonitor::writeBiasValue(int biasValue, int biasAddress) {
	input_sigs->s_input_bus_valid = 1;
	input_sigs->s_input_bus_data[0] = biasValue;
	input_sigs->s_input_bus_type = 0;
	input_sigs->s_input_bus_config_reg_addr[0] = biasAddress;
	writeToAxi();

}

void zs_driverMonitor::writeKernelValue(int kernelValue[2], int validMask) {
	input_sigs->s_input_bus_valid = validMask;
	input_sigs->s_input_bus_data[0] = kernelValue[0];
	input_sigs->s_input_bus_data[1] = kernelValue[1];
	input_sigs->s_input_bus_type = 2;
	writeToAxi();
}

void zs_driverMonitor::writePixels(int pos0, int pos1, bool pos1_valid,
		int instruction[2]) {
	input_sigs->s_input_bus_valid = (pos1_valid ? 3 : 1);
	input_sigs->s_input_bus_data[0] = pos0;
	input_sigs->s_input_bus_data[1] = (pos1_valid ? pos1 : 0);
	input_sigs->s_input_bus_type = 1;
	input_sigs->s_input_bus_config_reg_addr[0] = instruction[0];
	input_sigs->s_input_bus_config_reg_addr[1] = instruction[1];
	writeToAxi();
}

uint16_t zs_driverMonitor::int_to_short(int data) {

	uint16_t newData;

	newData = (uint16_t) data;
	if (data < 0) {
		newData = (uint16_t) ~(data - 1);
		newData = (~newData) + 1;
	}
	return (newData);

}

void zs_driverMonitor::writeToAxi() {

	unsigned int axiWord[2];
	memset(axiWord, 0, sizeof(unsigned int) * 2);

	axiWord[0] =
			((unsigned int) int_to_short(input_sigs->s_input_bus_data[0])
					| (unsigned int) int_to_short(
							input_sigs->s_input_bus_data[1]) << 16);

	axiWord[1] = (unsigned int) int_to_short(input_sigs->s_input_bus_type)
			| ((unsigned int) int_to_short(input_sigs->s_input_bus_valid) << 2)
			| ((unsigned int) int_to_short(
					input_sigs->s_input_bus_config_reg_addr[0]) << 4)
			| ((unsigned int) int_to_short(
					input_sigs->s_input_bus_config_reg_addr[1]) << 11);

	if (m_firstAxiWrite) {
		axiWord[1] |= (1 << 18);
		axiWord[1] |= (MAX_BURST << 19); //(m_maxOutPixelsNum << 19);
		m_firstAxiWrite = false;
	}

	if (current_write >= MEM_SIZE - 2) {
		axiWriteCommit();
	}

	virtual_source_address_[current_write] = axiWord[0];
	virtual_source_address_[current_write + 1] = axiWord[1];
	current_write += 2;

}

void zs_driverMonitor::axiWriteCommit() {

	if (current_write == 0)
		return;

	unsigned int size_int = sizeof(int);

	unsigned int padding = (MAX_BURST * (TRANSLEN_ / size_int))
			- (current_write % (MAX_BURST * (TRANSLEN_ / size_int)));
	int loPadding = virtual_source_address_[current_write - 2];
	int hiPadding = virtual_source_address_[current_write - 1] & (~(12)); //set valid bits to zero

	for (unsigned int i = 0; i < padding / 2; ++i) {
		virtual_source_address_[current_write] = loPadding;
		virtual_source_address_[current_write + 1] = hiPadding;
		current_write += 2;
	}

	for (unsigned int i = 0; i < current_write;
			i += MAX_BURST * (TRANSLEN_ / size_int)) //we are writing MAX_BURST*TRANSLEN bytes per iteration
					{
		unsigned int startPos = i * size_int; //start position for transfer in bytes

		//unsigned int burst_size = min(MAX_BURST * TRANSLEN_,
		//		(current_write - i) * size_int);

		//if (burst_size != MAX_BURST * TRANSLEN_)
		//	fprintf(stderr, "error in padding\n");

		writeAxiCommit_(MAX_BURST, startPos);

	}

	current_write = 0;

}

int zs_driverMonitor::writeAxiCommit_(int wordsNumber, unsigned int startPos) {

	//printf("%p \n", (void*)virtual_address_);

	int numbytes = 0;

	if (wordsNumber > 0 || wordsNumber <= 64) {

		//dma_s2mm_sync_halted_and_notIDLE_(virtual_address_);

		// Set destination and source addresses
		//dma_set(virtual_address, S2MM_DESTINATION_ADDRESS, dest_addr_offset);
		dma_set_(virtual_address_, MM2S_START_ADDRESS_,
		src_addr_offset_ + startPos);

		// Enable interruptions and start S2MM and MM2S
		//dma_set(virtual_address, S2MM_CONTROL_REGISTER, 0xf001);
		dma_set_(virtual_address_, MM2S_CONTROL_REGISTER_, 0x0001); //0xf001

		// Set tranference length for S2MM and MM2S. S2MM must be set before MM2S. In this point the tranferece starts
		//dma_set(virtual_address, S2MM_LENGTH, TRANSLEN*wordsNumber);
		dma_set_(virtual_address_, MM2S_LENGTH_, TRANSLEN_ * wordsNumber);

		dma_mm2s_sync_(virtual_address_);

		//dma_set_(virtual_address_, MM2S_CONTROL_REGISTER_, 0); // Stop MM2S
		dma_set_(virtual_address_, MM2S_STATUS_REGISTER_, 2); // Clear idle
		dma_set_(virtual_address_, MM2S_STATUS_REGISTER_, 0x1000); // Clear IOC_Irq

		numbytes = TRANSLEN_ * wordsNumber;
	}

	return (numbytes);
}

unsigned int zs_driverMonitor::reverseInt(unsigned int src) {
	unsigned int res = 0;
	res |= (src & (255 << 24)) >> 24;
	res |= (src & (255 << 16)) >> 8;
	res |= (src & (255 << 8)) << 8;
	res |= (src & 255) << 24;
	return (res);
}

void zs_driverMonitor::stopS2MM_() {
	dma_set_(virtual_address_, S2MM_CONTROL_REGISTER_, 0); // Stop S2MM
	dma_set_(virtual_address_, S2MM_STATUS_REGISTER_, 2); // Clear idle
	dma_set_(virtual_address_, S2MM_STATUS_REGISTER_, 0x1000); // Clear IOC_Irq
}

void zs_driverMonitor::readFromAxi() {

	//printf("sem %d\n", signal_done);
	if (m_gotAllPixels) {
		return;
	}
	waitValidAxiDataToRead_(MAX_BURST);
	unsigned int size_int = sizeof(int);
	for (unsigned int i = 0; i < MAX_BURST * (TRANSLEN_ / size_int); i += 2) {

		output_sigs->s_output_pixel_stream = virtual_destination_address_[i];
		output_sigs->s_output_pixel_stream_valid =
				(virtual_destination_address_[i + 1] & 3);

		pixel_step(); // get pixels and decode

	}

}

void zs_driverMonitor::pixel_step() {

	//printf("inside pixel_step, m_gotAllPixels: %d \n",m_gotAllPixels);

	if (m_gotAllPixels) {
		return;
	}

	unsigned int unpooled_outputHeight = (m_hinMax - m_hk + 1
			+ m_inputLayerPadding * 2);
	unsigned int unpooled_outputWidth = m_imageWidth - m_wk + 1
			+ m_inputLayerPadding * 2;

	unsigned int outputHeight = unpooled_outputHeight;
	unsigned int outputWidth = unpooled_outputWidth;
	if (m_poolingEnabled) {
		outputHeight /= 2;
		outputWidth /= 2;
	}

	if ((output_sigs->s_output_pixel_stream_valid) != 0) {

		for (unsigned int decode_iter = 0; decode_iter < 2; ++decode_iter) {
			int decoded_value = (
					decode_iter == 0 ?
							output_sigs->s_output_pixel_stream
									& (ipow(2, 16) - 1) :
							(output_sigs->s_output_pixel_stream >> 16));

			if ((m_currentDecodeSM == 0) && (m_encodingEnabled == 1)) // when encoding is enabled. Only when SM is all zeros, i.e. when all pixels have been read.
					{
				m_currentDecodeSM = decoded_value;
				m_currentDecodeIndex += 16; // initialized to -16, here put back to zero

				if (((m_currentDecodeIndex + 16)
						== ((outputHeight * outputWidth * m_nchOut)))
						&& (m_currentDecodeSM == 0)) {
					printf("got all layer pixels (SM all zeros) %d \n",
							outputHeight * outputWidth * m_nchOut);
					m_gotAllPixels = true;
					//  *sw_resetn = 0; // TODO: try to reset fpga here
					return;
				}

				tempPos = m_currentDecodeIndex;

			}

			else // if SM has any entry non zero, both for encoding enabled or disabled.
			{
				if ((m_currentDecodeSM == 0) && (m_encodingEnabled == 0)) { // only when SM is all zeros, when encoding is DISABLED.
					m_currentDecodeIndex += 16;
					tempPos = m_currentDecodeIndex;
					m_currentDecodeSM = (ipow(2, 16) - 1);
				}

				// assign value to outputPixel from one of the two halves of the output stream.
				int outputPixel = decoded_value;
				if (decoded_value & (1 << 15))
					outputPixel |= ((ipow(2, 16) - 1) << 16);
				else
					outputPixel &= ((ipow(2, 16) - 1));

				unsigned int MSB_one = 0;

				//printf("before shifthng (1 << MSB_one) %d\n", (1 << MSB_one) );

				while (!((1 << MSB_one) & m_currentDecodeSM))
					++MSB_one;

				//printf("after shifting (1 << MSB_one) %d\n", (1 << MSB_one) );

				m_currentDecodeSM &= ~((1 << MSB_one));

				//printf("(1 << MSB_one) %d,m_currentDecodeSM %d \n", (1 << MSB_one), m_currentDecodeSM );

				tempPos = m_currentDecodeIndex + MSB_one;

				// part needed to print the position of the output pixel (even if already calculated)
				bottomRow = false;

				if (!m_poolingEnabled) {
					if ((tempPos / m_nchOut) % 2) //bottom Row
							{
						tempPos -= m_nchOut;
						bottomRow = true;
					}

					shift = tempPos % m_nchOut;
					tempPos = (tempPos - shift) / 2 + shift;
				}

				// get the position of the output pixel to be generated from tempPos.
				chIdx = tempPos % m_nchOut + m_outputChannelOffset;
				tempPos /= m_nchOut;
				xPos = tempPos % outputWidth;
				tempPos /= outputWidth;
				yPos = tempPos * (m_poolingEnabled ? 1 : 2)
						+ (bottomRow ? 1 : 0);

				m_outputImage[chIdx][yPos][xPos] = outputPixel;

				if ((m_currentDecodeSM == 0)
						&& ((m_currentDecodeIndex + 16)
								== (outputHeight * outputWidth * m_nchOut))) {
					printf("got all layer pixels %d \n",
							outputHeight * outputWidth * m_nchOut);
					m_gotAllPixels = true;

				}

			}
			if (output_sigs->s_output_pixel_stream_valid == 1) {
				break;
			}
		}

	}

}

void zs_driverMonitor::phase2_step() {
	//write the kernel registers
	input_sigs->s_output_pixel_stream_enable = 1;

	if (!m_completedConfigWrite) {
		m_completedConfigWrite = initializationLoop();
	}

	else if (!m_completedBiasWrite) {

		if (!(m_biasWritePos % m_macs_per_channel)) {
			writeBiasValue(
					m_biases[m_biasWritePos / m_macs_per_channel
							+ m_outputChannelOffset], m_biasWritePos);
		} else {
			writeBiasValue(0, m_biasWritePos);
		}

		m_biasWritePos++;

		m_completedBiasWrite = (m_biasWritePos >= NUM_MAC_BLOCKS);

	}

	else if (!m_completedKernelWrite) {

		int nKernels = 0;
		int kernel[2];
		kernel[0] = kernel[1] = 0;
		for (; nKernels < 2; ++nKernels) {
			unsigned int tempPos = m_kernelWritePos;

			unsigned int xPoss = tempPos % m_wk;
			tempPos /= m_wk;

			unsigned int yPoss = tempPos % m_hk;
			tempPos /= m_hk;

			unsigned int srcChPos = tempPos % m_nchIn_pseudo;
			tempPos /= m_nchIn_pseudo;

			unsigned int dstChPos = tempPos + m_outputChannelOffset;

			int pseudo_ratio = m_nchOut_pseudo / m_nchOut;
			if (!((int) dstChPos % (int) pseudo_ratio)
					&& (int) srcChPos < (int) m_nchIn) {
				kernel[nKernels] =
						m_kernelArray[dstChPos / pseudo_ratio][srcChPos][yPoss][xPoss];
			}

			else {
				kernel[nKernels] = 0;
			}

			++m_kernelWritePos;
			if (m_kernelWritePos % m_contiguous_kernels == 0)
				break;
		}

		if (nKernels == 0) {
			writeKernelValue(kernel, 1);
		} else {
			writeKernelValue(kernel, 3);
		}

		m_completedKernelWrite = ((int) m_kernelWritePos
				== (int) (m_nchIn_pseudo * m_nchOut_pseudo * m_wk * m_hk));

	}

	else if ((int) m_kernelWritePos
			== (int) (m_nchIn_pseudo * m_nchOut_pseudo * m_wk * m_hk)) {
		sendConfigData(config_kernel_memory_write_complete_pulse, 1);
		++m_kernelWritePos;
	}

	else if (!m_sent_start_pulse) {
		sendConfigData(config_start_process_pulse, 1);
		m_sent_start_pulse = true;
	}

	else if (!m_sent_image_ready && m_currentInputPass != 0) {
		sendConfigData(CONFIG_TYPE(20), 1);
		m_completedImageWrite = true;
		m_sent_image_ready = true;
	}

	else if (!m_completedImageWrite) {
		if (!m_imageCompressionEnabled) {
			int pixels[2];
			bool pixel2Valid = true;
			for (unsigned int iter = 0; iter < 2; ++iter) {
				unsigned int tempPos = m_imageWritePos;

				unsigned int srcChannel = tempPos % m_nchIn;
				tempPos /= m_nchIn;

				unsigned int xPoss = tempPos % m_imageWidth;
				tempPos /= m_imageWidth;

				unsigned int yPoss = tempPos;
				if (srcChannel == 0 && xPoss == 0) {
					instruction[iter] = 15;
					old_row = yPos;
				}

				pixels[iter] = m_image[srcChannel][yPoss][xPoss];
				++m_imageWritePos;
				m_completedImageWrite = ((int) m_imageWritePos
						>= (int) (m_nchIn * m_hinMax * m_imageWidth));
				if (m_completedImageWrite && iter == 0) {
					pixel2Valid = false;
					break;
				}
			}

			writePixels(pixels[0], pixels[1], pixel2Valid, instruction);
			memset(instruction, 0, 2 * sizeof(int));

		}

		else {

			bool onePixelRemaining = false;
			int remainingPixel;
			bool done = ((int) m_imageWritePos
					>= (int) (m_nchIn * m_hinMax * m_imageWidth));
			if (m_pixelArrayWritePos + 1 < m_nPixelsArray) {
				writePixels(m_pixelArray[m_pixelArrayWritePos],
						m_pixelArray[m_pixelArrayWritePos + 1], true,
						instruction);
				memset(instruction, 0, 2 * sizeof(int));

				m_pixelArrayWritePos += 2;
				if (done && (m_pixelArrayWritePos == m_nPixelsArray))
					m_completedImageWrite = true;
			}

			else if ((m_pixelArrayWritePos == (m_nPixelsArray - 1)) && done) {
				writePixels(m_pixelArray[m_pixelArrayWritePos], 0, false,
						instruction);
				memset(instruction, 0, 2 * sizeof(int));

				m_completedImageWrite = true;
			}

			else {
				unsigned int SM = 0;
				if (m_pixelArrayWritePos < m_nPixelsArray) {
					onePixelRemaining = true;
					remainingPixel = m_pixelArray[m_pixelArrayWritePos];
				}

				for (int empty_iter = 0; empty_iter < 2; ++empty_iter) {
					m_nPixelsArray = m_pixelArrayWritePos = 0;
					for (int iter = 0; iter < 16; ++iter) {
						unsigned int tempPos = m_imageWritePos;

						unsigned int srcChannel = tempPos % m_nchIn;
						tempPos /= m_nchIn;

						unsigned int xPoss = tempPos % m_imageWidth;
						tempPos /= m_imageWidth;

						unsigned int yPoss = tempPos;
						done = ((int) m_imageWritePos
								>= (int) (m_nchIn * m_hinMax * m_imageWidth));

						if (srcChannel == 0 && xPoss == 0) {
							instruction[empty_iter] = 15;
							old_row = yPoss;
						}

						if (done) {
							++m_imageWritePos;
							break;
						}

						if (srcChannel == 0 && xPoss == 0 && yPoss != 0
								&& iter != 0)
							break;

						int pixel = m_image[srcChannel][yPoss][xPoss];

						if (pixel != 0) {
							m_pixelArray[m_nPixelsArray++] = pixel;
							SM |= (1 << iter);
						}

						++m_imageWritePos;
					}

					if (onePixelRemaining) {
						instruction[1] = instruction[0]; //we are in empty_iter=1
						instruction[0] = 0;
						writePixels(remainingPixel, SM, true, instruction);
						memset(instruction, 0, 2 * sizeof(int));

						break; //from empty_iter
					}

					else if (empty_iter == 1) {
						if (SM == 0 && done)
							m_completedImageWrite = true;
						else {
							writePixels(0, SM, true, instruction);
							memset(instruction, 0, 2 * sizeof(int));

						}
					}

					else if (SM != 0) {
						writePixels(SM, m_pixelArray[m_pixelArrayWritePos],
						true, instruction);
						memset(instruction, 0, 2 * sizeof(int));

						++m_pixelArrayWritePos;
						break; //from empty_iter
					}
				}
			}

		}
	} else if (!m_wroteImageDone) {
		sendConfigData(config_input_image_done, 1);
		m_wroteImageDone = true;
	} else
		axiWriteCommit();
}

int zs_driverMonitor::threadExists() {
	if (pthread_kill(m_readThread, 0) != 0) {
		launchThread();
	}
}

int zs_driverMonitor::processingLoop(unsigned int currentStep) {

	if (currentStep == 0) {
		gettimeofday(&start, NULL);
		counter_l = 0;
	}

	if (!m_activeProcessing) {
		//printf("changing layer\n");

		if (m_currentLayer != 0) {
			start = end;
		}
		gettimeofday(&end, NULL);
		secs = end.tv_sec - start.tv_sec;
		usecs = end.tv_usec - start.tv_usec;
		mtime = ((secs) * 1000 + usecs / 1000.0) + 0.5;
		time_for_eval_layer[m_currentLayer - 1] = mtime;
		if (setCurrentLayer(m_currentLayer) == FINISHED) {
			return (FINISHED);
		}

		setInternalVariables();
		initializeConfigArray();

	}

	phase2_step();

	m_activeProcessing = true;

	if (m_completedImageWrite && m_gotAllPixels) {
		m_activeProcessing = false;
		++m_currentInputPass;
		if (m_currentInputPass == m_numInputPasses) {
			m_currentInputPass = 0;
			++m_currentLayer;
		}
		if (m_currentLayer == m_numLayers) {

			struct timeval start, end;
			long mtime, secs, usecs;
			gettimeofday(&start, NULL);
			evaluateFCLayers();
			gettimeofday(&end, NULL);
			secs = end.tv_sec - start.tv_sec;
			usecs = end.tv_usec - start.tv_usec;
			mtime = ((secs) * 1000 + usecs / 1000.0) + 0.5;
			time_for_eval_fc = mtime;

		}

	}

	return (1);
}

void zs_driverMonitor::printStatus() {

	fprintf(stderr, "\n######################################\n");

	for (int h = 0; h < m_numLayers; h++) {
		fprintf(stderr, "TIME FOR LAYER %d NULLHOP: %ld ms\n", h,
				time_for_eval_layer[h]);
	}

	fprintf(stderr, "TIME FOR FC on armv7 : %ld ms\n", time_for_eval_fc);

	fprintf(stderr, "\n *** \n Second FC layer output: \n");
	for (unsigned int i = 0; i < IP2_OP_SIZE; i++) {
		fprintf(stderr, "m_fc2_output[%d]: %d \n", i, m_fc2_output[i]);
	}

	fprintf(stderr, "\n");

}

void * readThreadRoutine(void * arg) {

	zs_driverMonitor *zsDM = ((zs_driverMonitor *) arg);
	while (1) {
		try {
			zsDM->readFromAxi();
			usleep(15);
		} catch (const char *p) {
			std::cout << "caught" << p << "\n";
		}
	}

}
