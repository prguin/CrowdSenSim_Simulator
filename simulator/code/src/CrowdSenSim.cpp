//============================================================================
// Name        : CrowdSenSim.cpp

// Author      : Piergiorgio Vitello
// Version     : 1.2.0
// Copyright   : Your copyright notice
// Description : Crowd Sensing Simulator
//============================================================================
#include <iostream>
#include "../Headers/Position.h"
#include "../Headers/Location.h"
#include "../Headers/Sample.h"
#include "../Headers/Antenna.h"
#include "../Headers/Event.h"
#include "../Headers/ReadValues.h"
#include "../Headers/Simulation.h"
#include "../Headers/Smartphones.h"
#include "../Headers/Statistics.h"
#include "../Headers/User.h"
#include "../Headers/Utilities.h"
#include "../Headers/HeatMap.h"

using namespace std;

Smartphones smM;
Smartphones::iterator it_smM;

Users usersM;
Users::iterator it_usersM;

UsersStat usersstatM;
UsersStat::iterator it_usersstatM;

Antennas antennasL;
Antennas::iterator it_antennasL;



map <time_t,vector<Event> > eventsL;
Events eventsL2;

Events::iterator it_eventsL;



map <int,list<int> > usercon;
map <int,list<int> >::iterator it_m;

int days;
int num_users;


int main(int argc, char** argv) {

srand((unsigned)time(NULL));

cout << "*********************Start CrowdSenSim*********************" << endl;

num_users = readNumberUsers();

string typeOfAntennas;


Smartphones *psmM;
psmM = &smM;



Users *pusersM;
pusersM = &usersM;



time_t t0 = time(0);   // get time now
struct tm * now = localtime( & t0 );
cout << (now->tm_hour) << '-'
     << (now->tm_min) << '-'
     <<  now->tm_sec
     << endl;





/**
 * Decision for creating or not a new list of events.
 */
string fname;
string antennaFilename;
int decision,statdec;
decision = readDecision();
statdec=readStatDec();


//created a new list event so read user and days from setup
if(decision == 1){
	num_users = readNumberUsers();
	days = readDaySimulation();
	fname="./data/Inputs/Mobility/UserMovementsListEvents_";
	antennaFilename="./data/Inputs/CoordinatesAntennas.txt";
}


// we use the default list of event with fixe number of user 2000  and fixed number of days 2
if(decision == 0){

	num_users = 2000;
	days = 2;
	fname="./data/Inputs/Default/Mobility/UserMovementsListEventsDefault_";
	antennaFilename="./data/Inputs/CoordinatesAntennasLuxembourg.txt";
}

if(decision==2){

	fname="./data/Inputs/Default/Mobility/chosen_list/UserMovementsListEvents_";
	antennaFilename="./data/Inputs/Default/Mobility/chosen_list/CoordinatesAntennas.txt";
	ifstream set("./data/Inputs/Default/Mobility/chosen_list/Setup.txt");
	string s;
	getline(set,s);
	stringstream(s) >> num_users >> days ;
	set.close();

}


typeOfAntennas = readKindAntennaFromSetupFile();
antennasL = mapAntenna(antennaFilename, typeOfAntennas);

map <int , pair<Location,Location> > grid;
map <int ,vector<Location> > poicords;
int numcoluns;
vector<float> results;

int gfa=argc;// 1= static 2 = POI 3= Dynamic

*psmM = setSmartphones(num_users);

eventsL = readListOfEvents(fname,statdec,&usersM,days,num_users,&eventsL2,&grid,&poicords,&numcoluns,psmM);


//SIMULATION
if(gfa==1)
	results= simulationOperations(num_users,smM, it_smM,eventsL, usersM, it_usersM,days,eventsL2,it_eventsL,grid);

if(gfa==2)
	results= simulationOperationsPOI(num_users,smM, it_smM,eventsL, usersM, it_usersM,days,eventsL2,it_eventsL,grid,poicords,numcoluns);

if(gfa==3)
	results= simulationOperationsDYN(num_users,smM, it_smM,eventsL, usersM, it_usersM,days,eventsL2,it_eventsL,grid,numcoluns);

computeStatistics(num_users,smM,it_smM,eventsL2,it_eventsL,usersM,it_usersM,days,(*antennasL.begin()).loc,results);



//heatMapAntennas(num_users, eventsL, antennasL);
/* - - - - - - - - - - - - - - - */
// CLEANING STRUCTURES
/* - - - - - - - - - - - - - - - */
antennasL.clear();
eventsL.clear();





return 0;
}

