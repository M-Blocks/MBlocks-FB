/*
 * sensors.h
 *
 * Created: 4/10/2015 1:02:20 PM
 *  Author: kwgilpin
 */ 


#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>
#include <stdbool.h>

void sensors_enable(void);
void sensors_disable(void);
bool sensors_getEnabled(void);

#endif /* SENSORS_H_ */