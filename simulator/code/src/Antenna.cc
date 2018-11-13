/*
 * Antenna.cc

 *
 *  Created on: 30 giu 2016
 *      Author: Giuseppe Cacciatore
 */
#include "../Headers/Antenna.h"

#include <string>
#include <fstream>
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

Antennas antennasLFunc;
Antennas::iterator it_antennasLFunc;

Antennas mapAntenna(string textFileAntennasLocations, string typeOfAntennas){
/**
 * It sets antennas in the city map
 */
cout << "**Set up antennas system**"<<endl;
int id_ant=0;

/*
 * Constants of delimitation useful for reading the antennas file
 */
char const row_delim = '\n';
char const field_delim = ' ';
typedef vector<vector<string> > Rows;
Rows rows;

ifstream antenna_file(textFileAntennasLocations.c_str());
string dummyLine;
getline(antenna_file,dummyLine);
	for (string row; getline(antenna_file, row, row_delim); ){
		rows.push_back(Rows::value_type());
		istringstream ss(row);
		for (string field; getline(ss, field, field_delim); ){
			rows.back().push_back(field);
		}
		Location temp_loc(atof(rows.back().at(1).c_str()),atof(rows.back().at(2).c_str()),0.0);
		id_ant++;
		Antenna temp_ant(id_ant, typeOfAntennas,temp_loc);

		antennasLFunc.push_back(temp_ant);
	}
	if (!antenna_file.eof()){
		cerr << "Error in reading the file of Antennas locations!"<<endl;
	}
	antenna_file.close();

	ofstream antennamapfile;
	antennamapfile.open("../data/Outputs/CreatedMaps/AntennasCityMap/AntennasMap.txt");

	antennamapfile<<"/SystemUsed/-"<<"/ID-Antenna/-"<<"/Lat/-" << "/Long/"<<endl;

	for (it_antennasLFunc= antennasLFunc.begin(); it_antennasLFunc != antennasLFunc.end(); ++it_antennasLFunc){

			antennamapfile << (*it_antennasLFunc).type_antenna << "\t" << (*it_antennasLFunc).id_antenna << "\t"
						   << (*it_antennasLFunc).loc.lat<<"\t" << (*it_antennasLFunc).loc.lon << endl;
	}
	antennamapfile.close();
	antenna_file.close();
	return antennasLFunc;
}
