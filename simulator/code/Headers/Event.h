/*
 * Event.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#ifndef EVENT_H_
#define EVENT_H_

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
#include "../Headers/Location.h"
#include "../Headers/Utilities.h"
#include "../Headers/ClockManagement.h"

using namespace std;

// Structure for the list of events
// - - - - - - - - - - - - -
// the event corresponds to a user arriving in a given position (point)
// of the map at a given time
struct Goaielement
 {
      int goai;
      int id;

      Goaielement(int g,int i):goai(g),id(i){}

      bool operator <(const Goaielement& boh) const
      {
          if(goai== boh.goai)
        	  return id>boh.id;

          else
        	  return goai>boh.goai;
      }
      //version 1



 };

struct Event{

	int id_user;                /**< User ID */
	Location loc;               /**< Location of the event */
	tm timestamp;              /**< Time of the event */
	int quad;
	float speed;
	string status;
	int idgo;
	set <Goaielement> gset;
	int members;
	int groupcontinue;
	float distgo;
	int rssi;
    Event(int A, Location B, tm C,int q,float s,string st,int idg,set<Goaielement> gm,int membs,int gr, float gc,int ri):id_user(A), loc(B), timestamp (C), quad(q),speed(s),status(st),idgo(idg),gset(gm),members(membs),groupcontinue(gr),distgo(gc),rssi(ri){}
};

typedef std::list<Event> Events;






/**
 * It compares two Events
 * Inputs: Events lhs and Events rhs
 * Output: boolean value of the comparison
 */
bool eventComparator(const Event& lhs, const Event& rhs);

/**
 * It creates lists of events, ordered by UserID and time
 */

/**
 * It creates lists of events, ordered by UserID and time slots
 */

/**
 * It reads lists of events
 *
 */
map <time_t,vector<Event> > readListOfEvents (string fname,int dec,Users* pusersM,int days,int nusrs,Events* eventsL2,map <int , pair<Location,Location> >* grid,map <int ,vector<Location> >* poicords,int* numcoluns,Smartphones *psmM);


Events sortEvents(Events eves);

#endif /* EVENT_H_ */
