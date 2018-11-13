/*
 * Sample.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#ifndef SAMPLE_H_
#define SAMPLE_H_

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

#include "Position.h"

using namespace std;

struct Sample{

	string type;            /**< Type of the sensor */
	float value;            /**< Value of data */
	float size;             /**< Size of data to be delivered (Bytes) */
	bool ifsent;            /**< Sent or no */
	Position pos;           /**< Geo-temporal position */

	Sample(string A, float B, float C, bool D, Position E): type (A), value(B), size(C), ifsent(D), pos(E){}
};

typedef std::list<Sample> Samples;


#endif /* SAMPLE_H_ */
