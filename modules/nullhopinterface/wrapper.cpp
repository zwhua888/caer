/* C wrapper to NullHop interface
 *  Author: federico.corradi@inilabs.com
 */
#include "classify.hpp"
#include "wrapper.h"

extern "C" {

zs_driverMonitor* newzs_driverMonitor() {
	return new zs_driverMonitor();
}

int zs_driverMonitor_loadImage(zs_driverMonitor* v){
	return v->loadImage();
}

int zs_driverMonitor_threadExists(zs_driverMonitor* v){
	return v->threadExists();
}

int zs_driverMonitor_launchThread(zs_driverMonitor* v){
	return v->launchThread();
}

void zs_driverMonitor_file_set(zs_driverMonitor* v, uint8_t * picture) {
	v->file_set(picture);
}

void zs_driverMonitor_initNet(zs_driverMonitor* v) {
	return v->initNet();
}

void zs_driverMonitor_resetAxiBus(zs_driverMonitor* v) {
	return v->resetAxiBus();
}

void deleteMyClass(zs_driverMonitor* v) {
	delete v;
}

void zs_driverMonitor_closeThread(zs_driverMonitor* v){
	return v->closeThread();
}


void loadFCParams(zs_driverMonitor* v) {
	return v->loadFCParams();
}

}
