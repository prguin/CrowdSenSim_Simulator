/*
 * ClockManagement.cc
 *
 *
 *  Created on: 30 giu 2016
 *      Author: Giuseppe Cacciatore
 */
#include "../Headers/ClockManagement.h"

#include <time.h>


tm controlTime(tm ref_time_, int minutes_trav_){
/**
 *	It checks the travel time
 */
int minutes=0;
if(ref_time_.tm_min + minutes_trav_ < 59){
	ref_time_.tm_min = ref_time_.tm_min + minutes_trav_;
}
if(ref_time_.tm_min + minutes_trav_ > 60){

	do{
	   ref_time_.tm_hour++;

	}while(minutes > 60);

	ref_time_.tm_min = minutes;
}
if(ref_time_.tm_min + minutes_trav_ == 60){
	ref_time_.tm_hour++;
	ref_time_.tm_min = 0;
}
return ref_time_;
}
