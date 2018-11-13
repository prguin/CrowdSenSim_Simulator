/*
 * Antenna.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#ifndef ANTENNA_H_
#define ANTENNA_H_

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

#include "Location.h"

struct Antenna{

	int id_antenna;             /**< Antenna ID */
    string type_antenna;        /**< Type of antenna */
	Location loc;              /**< Geographical position of the Antenna */

	Antenna(int A, string B, Location C):id_antenna(A),type_antenna(B), loc(C){}
};

typedef std::list<Antenna> Antennas;  /**< List of antenna */

/**
 * Creates antennas-map.txt file
 *
 * Inputs: -file of geographical location of Antennas
 * 		   -Name of the kind of antennas system
 *
 * Returns a list of Antenna
 *
 */
Antennas mapAntenna(string textFileAntennasLocations, string typeOfAntennas);

#endif /* ANTENNA_H_ */
