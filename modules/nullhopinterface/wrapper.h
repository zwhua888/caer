/* C wrapper to NullHop Zynq interface
 *  Author: federico.corradi@gmail.com
 */
#ifndef __WRAPPER_H
#define __WRAPPER_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct zs_driverMonitor zs_driverMonitor;

zs_driverMonitor* newzs_driverMonitor();

void zs_driverMonitor_file_set(zs_driverMonitor* v, char * i, double *b, double thr);

char * zs_driverMonitor_file_get(zs_driverMonitor* v);

void deleteMyClass(zs_driverMonitor* v);

void loadFCParams(zs_driverMonitor* v);

const char * caerNullHopWrapper(uint16_t moduleID, char ** file_string, double *classificationResults, int max_img_qty);

#ifdef __cplusplus
}
#endif
#endif

