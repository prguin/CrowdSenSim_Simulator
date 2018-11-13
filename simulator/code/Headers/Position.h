/*
 * Position.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#ifndef POSITION_H_
#define POSITION_H_

#include <time.h>

#include "Location.h"

struct Position{

	Location loc;             /**< Geographic location */
	tm timestamp;             /**< Time */

	Position(Location A, tm B): loc(A), timestamp(B){}
};


#endif /* POSITION_H_ */
