/*
 * Simulation.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "../Headers/Utilities.h"

/**
 * It creates the SimulationData.txt file useful to compute the statistics and it is the core of the simulator.
 */







vector<float> simulationOperations(int num_users,Smartphones smM, Smartphones::iterator it_smM,map <time_t,vector<Event> > contacts, Users usersM, Users::iterator it_usersM,int days,Events eventsL,Events::iterator it_eventsL,map<int,pair<Location,Location> >  grid);
vector<float> simulationOperationsPOI(int num_users,Smartphones smM, Smartphones::iterator it_smM,map <time_t,vector<Event> > contacts, Users usersM, Users::iterator it_usersM,int days,Events eventsL,Events::iterator it_eventsL,map<int,pair<Location,Location> >  grid,map <int ,vector<Location> > poicords,int numcoluns);
vector<float> simulationOperationsDYN(int num_users,Smartphones smM, Smartphones::iterator it_smM,map <time_t,vector<Event> > contacts, Users usersM, Users::iterator it_usersM,int days,Events eventsL2,Events::iterator it_eventsL2,map <int , pair<Location,Location> > grid,int numcoluns);
int updateGoai(float battery,Location loc,Samples s,float rssi, float speed, Location lb,Location lu);
int ClosestPoi(Location p,map <int, pair<Location,Location> > grid, vector<Location> vecpoi,int numpoipast);
int updateGoaiPoi(float battery,int minwait,Samples s,float realrssi, float speed, Location poi);
int GoElection(set<Goaielement>::iterator last ,map <int,vector<Event>::iterator > eves, vector<Event>::iterator currevent);
int computeCkt(Location loc1,Samples oldsmp1,Location loc2,Samples oldsmp2,float speed1,float speed2);

int updateGoaiDYN(int C,float battery,int rssi);
bool idgocomp (Event i,Event j);
#endif /* SIMULATION_H_ */
