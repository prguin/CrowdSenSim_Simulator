/*
 * User.cc

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#include "../Headers/User.h"

#include "../Headers/ReadValues.h"
#include "../Headers/Utilities.h"



//Users allocateUsers(int num_users, StreetPoint stpM,StreetPoint::iterator it_stpM){
///**
// * It allocates users in the map. The first position is chosen casually.
// */
//Users usersM;
//
//cout << "**Allocate users**"<<endl;
//for(int i=1; i<num_users; i++){
//
//	 int initial_pos=(int)fRand(1.0,stpM.size());
//	 it_stpM=stpM.find(initial_pos);
//	 Location temp_loc((*it_stpM).second.lat,(*it_stpM).second.lon,(*it_stpM).second.alt);
//
//	 tm initial_time;
//	 int startHour = readStartHour();
//	 int startMinute = readStartMinute();
//	 int finishHour = readFinishtHour();
//	 int finishMinute = readFinishMinute();
//
//	 initial_time.tm_hour =fRand(startHour,finishHour);
//	 initial_time.tm_min=fRand(startMinute,finishMinute);
//	 Position temp_pos(temp_loc,initial_time);
//	 User temp_str(i,initial_pos,temp_pos);
//	 usersM.insert(pair <int,User >(i,temp_str));
//}
//
//return usersM;
//}

//Users allocateUsersSlotMobility(int num_users_tot, StreetPoint stpM,StreetPoint::iterator it_stpM){
///**
// * It allocates users in different slots time, according to the presence probability. If it is higher there are many users in that slot.
// * There are 10 slots and each one lasts 1 hour.
// * If it chosen this kind of users allocation, the simulation starts at 21 and finish at 7.
// */
//cout << "**Allocate users using slot mobility**"<<endl;
//Users usersM;
//
//float probability_decreasing[] = {0.3,0.2,0.15,0.1,0.1,0.05,0.05,0.02,0.02,0.01};
//float probability_increasing[] = {0.01,0.02,0.02,0.05,0.05,0.1,0.1,0.15,0.2,0.3};
//float probability[]={0.3,0.1,0.05,0.04,0.02,0.04,0.05,0.1,0.3};
//int number_slots = 10;
//
//int first_users_slot=1;
//
//tm initial_time;
//initial_time.tm_hour = 21;
//
//for(int j=0;j<number_slots;j++){
//	 int num_users_slot;
//	 num_users_slot = num_users_tot*probability_decreasing[j];
//	 for(int i=first_users_slot; i<first_users_slot+num_users_slot; i++){
//		 int initial_pos=(int)fRand(1.0,stpM.size());
//		 it_stpM=stpM.find(initial_pos);
//		 Location temp_loc((*it_stpM).second.lat,(*it_stpM).second.lon,(*it_stpM).second.alt);
//		 initial_time.tm_wday=0;
//		 initial_time.tm_min=fRand(0,59);
//		 if((initial_time.tm_hour>=24)&&(initial_time.tm_min>0)){
//			  initial_time.tm_wday = initial_time.tm_wday + 1;
//			  initial_time.tm_hour = initial_time.tm_hour - 24;
//		 }
//		 Position temp_pos(temp_loc,initial_time);
//		 User temp_str(i,initial_pos,temp_pos);
//		 usersM.insert(pair <int,User >(i,temp_str));
//	 }
//	 initial_time.tm_hour=initial_time.tm_hour+1;
//	 first_users_slot=first_users_slot+num_users_slot;
//}
//
//return usersM;
//}
