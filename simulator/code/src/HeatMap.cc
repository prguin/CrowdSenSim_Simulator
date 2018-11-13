/*
 * HeatMap.cc
 *
 *  Created on: 04 nov 2016
 *      Author: giuseppe
 */
#include "../Headers/HeatMap.h"

void heatMapAntennas(int num_users,Events eventsL,Antennas antennasL){

	Antennas::iterator it_antennasL;
	Events::iterator it_eventsL;
	int numberOfUsersNearAntenna;
	/*
	 * Default value of antenna covered radius
	 */
	int radius = 40;
	ofstream heatMap;
	heatMap.open("../data/HeatMapPosition.txt");

	for (it_antennasL= antennasL.begin(); it_antennasL != antennasL.end(); ++it_antennasL){

		numberOfUsersNearAntenna=0;

		for(it_eventsL = eventsL.begin(); it_eventsL != eventsL.end(); ++it_eventsL){

			double meters   = havdist((*it_antennasL).loc.lat,(*it_antennasL).loc.lon,(*it_eventsL).loc.lat,(*it_eventsL).loc.lon)*1000;

			if(meters<=radius){

				numberOfUsersNearAntenna++;

			}
		}
		/*
		 * In "HeatMapPosition.txt" file there are just the antennas with "numberOfUsersNearAntenna" not equal to zero
		 */
		if(numberOfUsersNearAntenna==0){

			continue;

		}
    	std::ostringstream strs;
    	strs << (*it_antennasL).loc.lat;
    	std::string str = strs.str();

    	std::ostringstream strs1;
    	strs1 << (*it_antennasL).loc.lon;
    	std::string str1 = strs1.str();

    	string String = static_cast<ostringstream*>( &(ostringstream() << numberOfUsersNearAntenna) )->str();

    	/*
    	 * Format of the "HeatMapPosition.txt" file useful for Heat Map in the webpage
    	 */
    	heatMap<<"{location: new google.maps.LatLng(" + str + ", " + str1 + "), weight: " + String + "},"<<endl;
	}
	heatMap.close();
}


