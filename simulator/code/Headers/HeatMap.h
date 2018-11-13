#ifndef HEATMAP_H_
#define HEATMAP_H_


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
#include <time.h>

#include "../Headers/Utilities.h"

using namespace std;

/**
 * It creates the "HeatMapPosition.txt" file, useful for the Heat Map on the statistics webpage
 *
 * Inputs: -num_users: total number of users
 * 		   -eventsL: list of simulation events
 *		   -antennasL: list of antennas in the city
 *
 *
 */
void heatMapAntennas(int num_users,Events eventsL,Antennas antennasL);


#endif /* HEATMAP_H_ */
