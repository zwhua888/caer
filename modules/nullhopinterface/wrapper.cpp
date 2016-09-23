/* C wrapper to NullHop interface
 *  Author: federico.corradi@inilabs.com
 */
#include "classify.hpp"
#include "wrapper.h"

extern "C" {

zs_driverMonitor* newzs_driverMonitor() {
	return new zs_driverMonitor();
}

void zs_driverMonitor_file_set(zs_driverMonitor* v, char * i, double *b, double thr) {
	v->file_set(i, b, thr);
}

char * zs_driverMonitor_file_get(zs_driverMonitor* v) {
	return v->file_get();
}

void deleteMyClass(zs_driverMonitor* v) {
	delete v;
}

void loadFCParams(zs_driverMonitor* v){
	return v->loadFCParams();
}

}
