/*
 * Statistics.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#ifndef STATISTICS_H_
#define STATISTICS_H_

#include "../Headers/Utilities.h"

/**
 *  It computes the statistics parameters from the SimulationData.txt file
 */
void computeStatistics(int num_users,Smartphones smM, Smartphones::iterator it_smM,Events eventsL, Events::iterator it_eventsL, Users usersM, Users::iterator it_usersM,int days,Location centre,vector<float> results);

map <int,map<time_t,int>  > analyzeContacts(Events eventsL, Events::iterator it_eventsL,int radius,int numusr);

#endif /* STATISTICS_H_ */
