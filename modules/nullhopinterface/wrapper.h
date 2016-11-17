/* C wrapper to NullHop Zynq interface
 *  Author: federico.corradi@gmail.com
 */
#ifndef __WRAPPER_H
#define __WRAPPER_H
#include <stdint.h>
#include <libcaer/events/frame.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef struct zs_driverMonitor zs_driverMonitor;

zs_driverMonitor* newzs_driverMonitor();

void zs_driverMonitor_file_set(zs_driverMonitor* v, uint16_t * picture);

int zs_driverMonitor_threadExists(zs_driverMonitor* v);

int zs_driverMonitor_launchThread(zs_driverMonitor* v);

void zs_driverMonitor_initNet(zs_driverMonitor* v);

void zs_driverMonitor_resetAxiBus(zs_driverMonitor* v);

void zs_driverMonitor_closeThread(zs_driverMonitor* v);

int zs_driverMonitor_loadImage(zs_driverMonitor* v);

void deleteMyClass(zs_driverMonitor* v);

void loadFCParams(zs_driverMonitor* v);

//const char * caerNullHopWrapper(uint16_t moduleID,  caerFrameEventPacket imagestreamer);

#ifdef __cplusplus
}
#endif
#endif

