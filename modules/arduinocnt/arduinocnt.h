/*
 * arduino control via serial port
 *
 *  Created on: Nov 2016
 *      Author: federico.corradi@inilabs.com
 */

#ifndef ARDUINOCNT_H
#define ARDUINOCNT_H

#include "main.h"

#include <libcaer/events/polarity.h>

void caerArduinoCNT(uint16_t moduleID, int results, bool *haveimage);

#endif /* ARDUINOCNT_H*/
