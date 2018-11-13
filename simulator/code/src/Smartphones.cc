/*
 * Smartphones.cc

 *
 *  Created on: 30 giu 2016
 *      Author: Giuseppe Cacciatore
 */

#include "../Headers/Smartphones.h"

#include "../Headers/Utilities.h"

Smartphones setSmartphones(int num_users){
/*
 * It set features of every smartphone.
 */
cout << "**Set up smartphones**"<<endl;
Smartphones smM;


static const int arr[] = {3300,3000,3100,3075,2600,3000,2200,2800,2800,2550}; /* j7,s7,j5,oppoA53,j3,p9,p8lite,g5,s5,s6  */
vector<int> capacities (arr, arr + sizeof(arr) / sizeof(arr[0]) );


for(int i=1; i<(num_users+1); i++){
	 string cntx;
	 if(i%2==0){
		 cntx="outdoor";
	 }else{
		 cntx="indoor";
	 }
	 DatatbdelM temp_map;
	temp_map["Accellerometer"]=0.0;
	temp_map["GPS"]=0.0;
	temp_map["Total"]=0.0;
	temp_map["Minutes"]=0.0;
	temp_map["Energy"]=0.0;
	temp_map["consumption-data"]=0.0;
	temp_map["consumption-wifi"]=0.0;

	Samples temp_s;
	vector<pair<float, float> > route;  /**< define route of user*/
	vector<bool> generate;		/**< for each location of route generate or not */
	vector<int> timeroute;

	float battery_level=fRand(10.0,90.0);
	int randomIndex = rand() % capacities.size();

	float cap=(float)capacities[randomIndex];
	Smartphone temp_str(i,battery_level,battery_level,cap,cntx,temp_map,temp_map,temp_s,0,route,generate,true,true,0,0,0,0,timeroute,0,0);
	smM.insert(pair <int, Smartphone >(i,temp_str));

}

return smM;
}


