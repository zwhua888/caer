#ifndef __ZS_AXIDMA_CPP__
#define __ZS_AXIDMA_CPP__

#include "zsaxidmalib.hpp"

void* write_thread_routine(void* arg) {
	printf("Creating write thread...\n");
	ZS_axidma* zsaxidma = (ZS_axidma*) arg;
	while (zsaxidma->is_write_thread_running()) {
		if (zsaxidma->write_to_axidma() > 0) {
			//   printf("written to axi");
		}
	}
	//zsaxidma->write_to_axidma();
	printf("Destroying write thread...\n");
	pthread_exit(NULL);
}

ZS_axidma::ZS_axidma() :
		axidma(AXIDMA_DEVICE_DEFINE, SOURCE_ADDR_OFFSET_DEFINE,
				DESTINATION_ADDR_OFFSET_DEFINE), axigpio(AXIGPIO_DEVICE_DEFINE) {
	//read_layer_finish = false;
	pthread_mutex_init(&write_mxt, NULL);
	write_thread = 1;
	write_thread_running = false;
}

ZS_axidma::~ZS_axidma(void) {
	pthread_cancel(write_thread);
}

void ZS_axidma::init(unsigned int read_tranfer_length) {
	reset();
	axidma.init(read_tranfer_length);
	pthread_create(&write_thread, NULL, write_thread_routine, (void*) this);
	write_thread_running = true;
}

void ZS_axidma::reset(void) {
	axidma.reset();
	axigpio.configure_port_direction(0x0);
	axigpio.write(0x1);
}

void ZS_axidma::stop(void) {
	write_thread_running = false;
	usleep(100000);
	axidma.stop();
}

void ZS_axidma::write(std::vector<uint64_t> data) {
	//printf("Waiting for mutex to be locked...\n");
	pthread_mutex_lock(&write_mxt);
	//printf("Mutex obtained\n");
	write_data.push_back(data);
	//printf("(Main thread) write_data list num elements --> %d. is write thread running? --> %d\n", write_data.size(), running_thread);
	//printf("Releasing mutex...\n");
	pthread_mutex_unlock(&write_mxt);
	//printf("Mutex released\n");
	/*if(!running_thread)
	 {
	 pthread_create(&write_thread, NULL, write_thread_routine, (void*)this);
	 }*/
}

unsigned int ZS_axidma::write_to_axidma(void) {
	//running_thread = true;
	pthread_mutex_lock(&write_mxt);
	bool write_data_list_empty = write_data.empty();
	//pthread_mutex_unlock(&write_mxt);

	int write_return = 0;

	//while(!write_data_list_empty)
	if (!write_data_list_empty) {
		//pthread_mutex_lock(&write_mxt);
		try {
			write_return = axidma.write(write_data.front());
			write_data.pop_front();
			//printf("(Write thread) write_data list num elements --> %d\n", write_data.size());
		} catch (AXIDMA_timeout_exception& ex) {
			printf(ex.what());
			stop();
			init(axidma.get_read_transfer_length_bytes());
		} catch (std::bad_alloc& ba) {
			printf("bad_alloc caught: %s. List size --> %d\n", ba.what(),
					write_data.size());
		}
		write_data_list_empty = write_data.empty();

	}
	//running_thread = false;
	pthread_mutex_unlock(&write_mxt);
	return write_return;
}

unsigned int ZS_axidma::readLayer(std::vector<uint64_t> *layer_data) {
	do {
		try {
			axidma.read(layer_data);
		} catch (AXIDMA_timeout_exception& ex) {
			printf(ex.what());
			stop();
			init(axidma.get_read_transfer_length_bytes());
			break;
		}

	} while ((layer_data->data()[layer_data->size() - 1] & 0x8000000000000000)
			== 0); //Check if ZS is IDLE - using != 0 we dont need to shift the data saving 1 instruction

	return layer_data->size() * sizeof(uint64_t);

}

bool ZS_axidma::is_write_thread_running(void) {
	return write_thread_running;
}

#endif

