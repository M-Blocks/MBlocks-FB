/*
 * timer1.h
 *
 * Created: 4/10/2015 11:14:32 AM
 *  Author: kwgilpin
 */ 


#ifndef TIMER1_H_
#define TIMER1_H_

#include <stdint.h>
#include <stdbool.h>

void timer1_init(void (*overflowCallback)(void));
bool timer1_start(void);
bool timer1_stop(void);


#endif /* TIMER1_H_ */