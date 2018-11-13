/*
 * Location.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#ifndef LOCATION_H_
#define LOCATION_H_

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

using namespace std;


struct Location{

  float lat;                  /**< Latitude- Geo */
  float lon;                  /**< Longitude - Geo*/
  float alt;                  /**< Altitude - Geo */

  Location(float A, float B, float C): lat(A), lon(B), alt(C){}
  Location(){}
};



#endif /* LOCATION_H_ */
