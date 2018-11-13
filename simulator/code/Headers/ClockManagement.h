/*
 * ClockManagement.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#ifndef CLOCK_MANAGEMENT_H_
#define CLOCK_MANAGEMENT_H_

#include <time.h>

/**
 * Updates the current value of ref_time_tm
 *
 * Inputs: -ref_time_tm: time to update
 * 		   -minutes_trav_: minutes to add
 *
 * Returns an object of the struct tm
 *
 */
tm controlTime(tm ref_time_tm_, int minutes_trav_);




#endif /* CLOCK_MANAGEMENT_H_ */
