/*
 * User.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

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

#ifndef USER_H_
#define USER_H_

struct User{

	int id_smartphone;          /**< Smartphone ID */
	int minutes;                 /**< number of minutes */
	//Position pos;               /**< Geo-temporal position */

	User(int A, int B): id_smartphone(A), minutes(B){}
};

typedef std::map<int,User> Users;


struct UserStat{

  int tot_num_samples;          /**< # of samples generated */
  int tot_data_gen;             /**< Amount of data generated */
  int tot_energy;               /**< Total energy spent */
  double tot_en_tx;				/**< Total energy spent for transmission */

  UserStat(int A, int B, int C, double D): tot_num_samples(A), tot_data_gen(B), tot_energy(C), tot_en_tx(D){}
};

typedef std::map<int,UserStat> UsersStat;

/**
 *  It allocates the users in the map. More in details the user are allocated evenly in the timestamp set from User
 */
//Users allocateUsers(int num_users, StreetPoint stpM,StreetPoint::iterator it_stpM);

/**
 * It allocates the users in the map. More in details the number of users allocated depends on the probability of presence for every time slot
 */
//Users allocateUsersSlotMobility(int num_users_tot, StreetPoint stpM,StreetPoint::iterator it_stpM);


#endif /* USER_H_ */
