/*
 * ReadValues.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#ifndef READINGVALUE_H_
#define READINGVALUE_H_

#include "../Headers/Utilities.h"

/**
 * These functions read values useful to starting the Simulation from the Setup.txt file, located in the Input folder
 */


int readStatDec();
int readDaySimulation();
int readNumberUsers();
string readKindAntennaFromSetupFile();
int readDecision();
int readRay();
int readStartHour();
int readStartMinute();
int readFinishtHour();
int readFinishMinute();
int readMaximumTravelTime();
int readMinimumTravelTime();
int readTypeCons();
int readBatteryDecision();

#endif /* READINGVALUE_H_ */
