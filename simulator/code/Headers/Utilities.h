/*
 * Utilities.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <map>
#include <list>
#include <algorithm>
#include <iomanip>
#include <iterator>
#include<set>
#include <sstream>
#include <vector>

#include <time.h>
#include "../Headers/Position.h"
#include "../Headers/Location.h"
#include "../Headers/Sample.h"
#include "../Headers/Smartphones.h"
#include "../Headers/User.h"
#include "../Headers/Antenna.h"
#include "../Headers/Event.h"
#include "../Headers/slamac.h"


using namespace std;


#define MODULE  2147483647
#define MYA 16807
#define LASTXN  127773
#define UPTOMOD -2836
#define RATIO   0.46566128e-9


long rnd32(long seed);
double uniform(double a, double b, long *seed);
double fRand(double fMin, double fMax);

#define pi 3.14159265358979323846
#define R 6371
#define TO_RAD (3.1415926536 / 180)

/**
 * It computes the distance in meters between two different points, given the Latitude and Longitude coordinates.
 */
double havdist(double th1, double ph1, double th2, double ph2);
void destVincenty(double lat1, double lon1, double bearing, double dist,double *lat2out, double *lon2out);
/**
 *  It converts bytes values in KB/MB/GB automatically.
 */
std::string PrintByteUnit(long double bytes);

/**
 * It converts byte values in MiB.
 */
std::string PrintByteUnitInMiB(long double bytes);

#endif /* UTILITIES_H_ */
