/* C wrapper to NullHop interface
 *  Author: federico.corradi@inilabs.com
 */
#include "classify.hpp"
#include "wrapper.h"

extern "C" {

zs_driverMonitor* newzs_driverMonitor() {
	return new zs_driverMonitor();
}

void zs_driverMonitor_file_set(zs_driverMonitor* v) {
	v->file_set();
}

char * zs_driverMonitor_file_get(zs_driverMonitor* v) {
	return v->file_get();
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

void loadFCParams(zs_driverMonitor* v) {
	return v->loadFCParams();
}

}
