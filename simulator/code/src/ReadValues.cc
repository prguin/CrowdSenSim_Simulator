/*
 * ReadValues.cc

 *
 *  Created on: 30 giu 2016
 *      Author: Giuseppe Cacciatore
 */

#include "../Headers/ReadValues.h"

#include <string.h>
#include <sstream>








int readDaySimulation(){
/*
 * It reads how many days of simulation are set in the "Setup.txt" file.
 */

int days;
ifstream in("../data/Setup.txt");
	if(!in){
		cerr << "Errors in file 'Setup.txt' " << endl;
	}
	string dummyLine;
	getline(in,dummyLine);
	string s;
	getline(in,s);
	string a;
	a =(s.substr(23,28));
	stringstream(a) >> days;
	in.close();

return days;
}

int readNumberUsers(){
/*
 * It reads how many users are selected for the simulation in the "Setup.txt" file.
 */

int num_users;
ifstream in("../data/Setup.txt");
if(!in){
	cerr << "Errors in file 'Setup.txt' " << endl;
}
string dummyLine;
int i = 1;
while(i < 5){
	 getline(in,dummyLine);
	 i++;
}
string s;
getline(in,s);
string a;
a =(s.substr(20,26));
stringstream(a) >> num_users;
in.close();

return num_users;
}

string readKindAntennaFromSetupFile(){
/*
 * It reads which type of antennas are used in the simulation, chosen in the "Setup.txt" file.
 */

ifstream in("../data/Setup.txt");
if(!in){
	cerr << "Errors in file 'Setup.txt' " << endl;
}
string dummyLine;
int i = 1;
while(i < 17){
	 getline(in,dummyLine);
	 i++;
}
string s;
getline(in,s);
string typeOfAntennas =s.substr(21,30);
in.close();

return typeOfAntennas;
}
int readStatDec(){
	int decision;

	ifstream in("../data/Setup.txt");
	string dummyLine;
	int i = 1;
	while(i < 34){
		 getline(in,dummyLine);
		 i++;
	}
	string s;
	getline(in,s);
	string a;
	a =(s.substr(18,30));
	stringstream(a) >> decision;
	in.close();
	return decision;
}
int readDecision(){
/*
 * It reads the decision chosen in the "Setup.txt" file regarding the creating of new list of events.
 */

int decision;
ifstream in("../data/Setup.txt");

string dummyLine;
int i = 1;
while(i < 23){
	 getline(in,dummyLine);
	 i++;
}
string s;
getline(in,s);
string a;
a =(s.substr(18,30));
stringstream(a) >> decision;
in.close();

if(decision==1){
    ifstream f("../data/Inputs/Default/Mobility/chosen_list/UserMovementsListEvents_0.txt");
    if(f.good()){

    	f.close();
    	decision=2;
    }


}


return decision;
}

int readRay(){
/*
 * It reads the value of the ray useful for creating the "MapGraphPoints.txt" file.
 */

int ray;
ifstream in("../data/Setup.txt");

string dummyLine;
int i = 1;
while(i < 29){
	 getline(in,dummyLine);
	 i++;
}
string s;
getline(in,s);
string a;
a =(s.substr(38,47));
stringstream(a) >> ray;
in.close();

return ray;
}

int readStartHour(){
/*
 * It reads the selected start hour of simulation in "Setup.txt" file.
 */

int startHour;
ifstream in("../data/Setup.txt");

string dummyLine;
int i = 1;
while(i < 10){
	 getline(in,dummyLine);
	 i++;
}
string s;
getline(in,s);
string a;
a =(s.substr(26,47));
stringstream(a) >> startHour;
in.close();

return startHour;
}

int readStartMinute(){
/*
 * It reads the selected start minute of simulation set in "Setup.txt" file.
 */

int startMinute;
ifstream in("../data/Setup.txt");

string dummyLine;
int i = 1;
while(i < 11){
	 getline(in,dummyLine);
	 i++;
}
string s;
getline(in,s);
string a;
a =(s.substr(28,47));
stringstream(a) >> startMinute;
in.close();

return startMinute;
}

int readFinishtHour(){
/*
 * It reads the selected finish hour of simulation in "Setup.txt" file.
 */

int finishHour;
ifstream in("../data/Setup.txt");

string dummyLine;
int i = 1;
while(i < 13){
	 getline(in,dummyLine);
	 i++;
}
string s;
getline(in,s);
string a;
a =(s.substr(27,47));
stringstream(a) >> finishHour;
in.close();

return finishHour;
}

int readFinishMinute(){
/*
 * It reads the selected start minute of simulation set in "Setup.txt" file.
*/

int finishMinute;
ifstream in("../data/Setup.txt");

string dummyLine;
int i = 1;
while(i < 14){
	 getline(in,dummyLine);
	 i++;
}
string s;
getline(in,s);
string a;
a =(s.substr(29,47));
stringstream(a) >> finishMinute;
in.close();

return finishMinute;
}

int readMinimumTravelTime(){
/*
 * It reads the minimum travel time set in the "Setup.txt" file.
 */
int minimum_travel_time;
ifstream in("../data/Setup.txt");

string dummyLine;
int i = 1;
while(i < 7){
	 getline(in,dummyLine);
	 i++;
}
string s;
getline(in,s);
string a;
a =(s.substr(24,30));
stringstream(a) >> minimum_travel_time;
in.close();

return minimum_travel_time;
}

int readMaximumTravelTime(){
/*
 * It reads the maximum travel time set in the "Setup.txt" file.
 */

int maximum_travel_time;
ifstream in("../data/Setup.txt");

string dummyLine;
int i = 1;
while(i < 8){
	 getline(in,dummyLine);
	 i++;
}
string s;
getline(in,s);
string a;
a =(s.substr(24,30));
stringstream(a) >> maximum_travel_time;
in.close();

return maximum_travel_time;
}

int readTypeCons(){
	int c;
	ifstream in("../data/Setup.txt");
	string dummyLine;
	int i = 1;
	while(i < 38){
		 getline(in,dummyLine);
		 i++;
	}
	string s;
	getline(in,s);
	string a;
	a =(s.substr(27,31));
	stringstream(a) >> c;
	in.close();

	return c;
}


int readBatteryDecision(){
/*
 * It reads the decision of using real battery traces  in the "Setup.txt" .
 */

int decision;
ifstream in("../data/Setup.txt");

string dummyLine;
int i = 1;
while(i < 44){
	 getline(in,dummyLine);
	 i++;
}
string s;
getline(in,s);
string a;
a =(s.substr(18,30));
stringstream(a) >> decision;
in.close();


return decision;
}







