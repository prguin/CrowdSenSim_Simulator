/*
 * Statistics.cc

 *
 *  Created on: 30 giu 2016
 *      Author: Giuseppe Cacciatore
 */

#include "../Headers/Statistics.h"

#include "../Headers/Utilities.h"
#include "../Headers/ReadValues.h"

void computeStatistics(int num_users,Smartphones smM, Smartphones::iterator it_smM,Events eventsL, Events::iterator it_eventsL, Users usersM, Users::iterator it_usersM,int days,Location centre,vector<float> results){
/*
 * It computes the Statistics of the entire Simulation.
 * All data are saved in the "Statistics.txt" file.
 *
 */
cout<<"**Compute statistics**"<<endl;
/**
 * Delimiter constants for reading file.
 */
char const row_delim = '\n';

vector<UsersStat> usersstatM(days);
UsersStat::iterator it_usersstatM;

ofstream outputStatistics;
outputStatistics.open("../data/Statistics.txt");





string kindOfAntennas = readKindAntennaFromSetupFile();
int startHour = readStartHour();
int startMinutes = readStartMinute();
int finishHour = readFinishtHour();
int finishMinutes = readFinishMinute();

outputStatistics << centre.lat << "," << centre.lon << "," << days << "," << num_users << ","<< startHour << ":" << startMinutes << "," << finishHour << ":"
		         << finishMinutes << "," << kindOfAntennas;



/* - - - - - - - - - - - - - - - - - - - */
// STATISTICS
/* - - - - - - - - - - - - - - - - - - - */


	// * * Per-user and global statistic on amound of data generated and energy spent
	vector<long int> acc_tot_data(days, 0);
	vector<long int> tem_tot_data(days, 0);
	vector<long int> pre_tot_data(days, 0);
	vector<long int> acc_tot_num_samples(days, 0);
	vector<long int> tem_tot_num_samples(days, 0);
	vector<long int> pre_tot_num_samples(days, 0);


	ifstream inputsimdata( "../data/Outputs/SimulationData.txt");

	string dummyLine;
	getline(inputsimdata,dummyLine);

	for(string row; getline(inputsimdata, row, row_delim); ){





	    istringstream buf(row);
	    istream_iterator<string> beg(buf), end;

	    vector<string> tokens(beg, end);




		int currentDay = atoi(tokens[0].c_str());


			if(tokens[7].compare("Accelerometer")==0){
				acc_tot_num_samples[currentDay] += atoi(tokens[8].c_str());
				acc_tot_data[currentDay] += atoi(tokens[9].c_str());
			}else if(tokens[7].compare("GPS")==0){
				tem_tot_num_samples[currentDay] += atoi(tokens[8].c_str());
				tem_tot_data[currentDay] +=atoi(tokens[9].c_str());
			}else if(tokens[7].compare("Proximity")==0){
		 	pre_tot_num_samples[currentDay] += atoi(tokens[8].c_str());
		 	pre_tot_data[currentDay] +=atoi(tokens[9].c_str());
			}
			int ref_user=atoi(tokens[3].c_str());
			if(usersstatM[currentDay].count(ref_user)==0){
				UserStat tmp_str(atoi(tokens[8].c_str()),atoi(tokens[9].c_str()),atoi(tokens[12].c_str()),atof(tokens[13].c_str()));
				usersstatM[currentDay].insert(pair <int, UserStat >(ref_user,tmp_str));
			}else{
				it_usersstatM=usersstatM[currentDay].find(ref_user);//find the user
				int up_num_samples=(*it_usersstatM).second.tot_num_samples+atoi(tokens[8].c_str());
				int up_data_gen=(*it_usersstatM).second.tot_data_gen+atoi(tokens[9].c_str());
				int up_energy=(*it_usersstatM).second.tot_energy+atoi(tokens[12].c_str());
				double up_energy_tx=(*it_usersstatM).second.tot_en_tx+atof(tokens[13].c_str());
				usersstatM[currentDay].erase(ref_user);
				UserStat tmp_str(up_num_samples,up_data_gen,up_energy,up_energy_tx);
				usersstatM[currentDay].insert(pair <int, UserStat >(ref_user,tmp_str));
			}

	}
	if(!inputsimdata.eof()){
		cerr << "Error in reading the file 'SimulationData.txt'!"<<endl;
	}
	inputsimdata.close();

for(int currentDay = 0; currentDay < days; currentDay++){

	std::map<int,int> energyM; // energy spent for sensing
	std::map<int,int> energytxM; // energy spent for transmission
	std::map<int,int>::iterator it_energy;


	long double avg_user_num_samples=0.0,avg_user_data=0.0,avg_user_energy=0.0,avg_user_energy_tx=0.0;
	for(it_usersstatM= usersstatM[currentDay].begin(); it_usersstatM != usersstatM[currentDay].end(); ++it_usersstatM){

		avg_user_num_samples+=(*it_usersstatM).second.tot_num_samples; //updating statistic for num of samples generated
		avg_user_data+=(*it_usersstatM).second.tot_data_gen; //updating statistic for data
		avg_user_energy+=(*it_usersstatM).second.tot_energy; //updating statistic for energy
		avg_user_energy_tx+=(*it_usersstatM).second.tot_en_tx; //updating statistic for energy spent for transmission

		if((*it_usersstatM).second.tot_energy>0 && (*it_usersstatM).second.tot_energy<150){
			if(energyM.count(150)==0){
				energyM[150]=1; //new time -> new record in the map
			}else{
				energyM[150]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>150 && (*it_usersstatM).second.tot_energy<200){
		if(energyM.count(200)==0){
			 energyM[200]=1; //new time -> new record in the map
		}else{
			energyM[200]+=1; //record already inserted in the map
		}
		}else if((*it_usersstatM).second.tot_energy>200 && (*it_usersstatM).second.tot_energy<250){
			if(energyM.count(250)==0){
				energyM[250]=1; //new time -> new record in the map
			}else{
				energyM[250]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>250 && (*it_usersstatM).second.tot_energy<300){
			if(energyM.count(300)==0){
				energyM[300]=1; //new time -> new record in the map
			}else{
				energyM[300]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>300 && (*it_usersstatM).second.tot_energy<350){
			if(energyM.count(350)==0){
				energyM[350]=1; //new time -> new record in the map
			}else{
				energyM[350]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>350 && (*it_usersstatM).second.tot_energy<400){
			if(energyM.count(400)==0){
				energyM[400]=1; //new time -> new record in the map
			}else{
				energyM[400]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>400 && (*it_usersstatM).second.tot_energy<450){
			if(energyM.count(450)==0){
				energyM[450]=1; //new time -> new record in the map
			}else{
				energyM[450]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>450 && (*it_usersstatM).second.tot_energy<500){
			if(energyM.count(500)==0){
				energyM[500]=1; //new time -> new record in the map
			}else{
				energyM[500]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>500 && (*it_usersstatM).second.tot_energy<550){
			if(energyM.count(550)==0){
				energyM[550]=1; //new time -> new record in the map
			}else{
				energyM[550]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>550 && (*it_usersstatM).second.tot_energy<600){
			if(energyM.count(600)==0){
				energyM[600]=1; //new time -> new record in the map
			}else{
				energyM[600]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>600 && (*it_usersstatM).second.tot_energy<650){
			if(energyM.count(650)==0){
				energyM[650]=1; //new time -> new record in the map
			}else{
				energyM[650]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>650 && (*it_usersstatM).second.tot_energy<700){
			if(energyM.count(700)==0){
				energyM[700]=1; //new time -> new record in the map
			}else{
				energyM[700]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>700 && (*it_usersstatM).second.tot_energy<750){
			if(energyM.count(750)==0){
				energyM[750]=1; //new time -> new record in the map
			}else{
				energyM[750]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>750 && (*it_usersstatM).second.tot_energy<800){
			if(energyM.count(800)==0){
				energyM[800]=1; //new time -> new record in the map
			}else{
				energyM[800]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>800 && (*it_usersstatM).second.tot_energy<850){
			if(energyM.count(850)==0){
				energyM[850]=1; //new time -> new record in the map
			}else{
				energyM[850]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>850 && (*it_usersstatM).second.tot_energy<900){
			if(energyM.count(900)==0){
				energyM[900]=1; //new time -> new record in the map
			}else{
				energyM[900]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>900 && (*it_usersstatM).second.tot_energy<950){
			if(energyM.count(950)==0){
				energyM[950]=1; //new time -> new record in the map
			}else{
				energyM[950]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_energy>950 && (*it_usersstatM).second.tot_energy<1000){
			if(energyM.count(1000)==0){
				energyM[1000]=1; //new time -> new record in the map
			}else{
				energyM[1000]+=1; //record already inserted in the map
			}
		}
		if((*it_usersstatM).second.tot_en_tx>0 && (*it_usersstatM).second.tot_en_tx<5){
			if(energytxM.count(5)==0){
				energytxM[5]=1; //new time -> new record in the map
			}else{
				energytxM[5]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>5 && (*it_usersstatM).second.tot_en_tx<10){
			if(energytxM.count(10)==0){
				energytxM[10]=1; //new time -> new record in the map
			}else{
				energytxM[10]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>10 && (*it_usersstatM).second.tot_en_tx<15){
			if(energytxM.count(15)==0){
				energytxM[15]=1; //new time -> new record in the map
			}else{
				energytxM[15]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>15 && (*it_usersstatM).second.tot_en_tx<20){
			if(energytxM.count(20)==0){
				energytxM[20]=1; //new time -> new record in the map
			}else{
				energytxM[20]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>20 && (*it_usersstatM).second.tot_en_tx<25){
			if(energytxM.count(25)==0){
				energytxM[25]=1; //new time -> new record in the map
			}else{
				energytxM[25]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>25 && (*it_usersstatM).second.tot_en_tx<30){
			if(energytxM.count(30)==0){
				energytxM[30]=1; //new time -> new record in the map
			}else{
				energytxM[30]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>30 && (*it_usersstatM).second.tot_en_tx<35){
			if(energytxM.count(35)==0){
				energytxM[35]=1; //new time -> new record in the map
			}else{
				energytxM[35]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>35 && (*it_usersstatM).second.tot_en_tx<40){
			if(energytxM.count(40)==0){
				energytxM[40]=1; //new time -> new record in the map
			}else{
				energytxM[40]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>40 && (*it_usersstatM).second.tot_en_tx<45){
			if(energytxM.count(45)==0){
				energytxM[45]=1; //new time -> new record in the map
			}else{
				energytxM[45]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>45 && (*it_usersstatM).second.tot_en_tx<50){
			if(energytxM.count(50)==0){
				energytxM[50]=1; //new time -> new record in the map
			}else{
				energytxM[50]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>50 && (*it_usersstatM).second.tot_en_tx<55){
			if(energytxM.count(55)==0){
				energytxM[55]=1; //new time -> new record in the map
			}else{
				energytxM[55]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>55 && (*it_usersstatM).second.tot_en_tx<60){
			if(energytxM.count(60)==0){
				energytxM[60]=1; //new time -> new record in the map
			}else{
				energytxM[60]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>60 && (*it_usersstatM).second.tot_en_tx<65){
			if(energytxM.count(65)==0){
				energytxM[65]=1; //new time -> new record in the map
			}else{
				energytxM[65]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>65 && (*it_usersstatM).second.tot_en_tx<70){
			if(energytxM.count(70)==0){
				energytxM[70]=1; //new time -> new record in the map
			}else{
				energytxM[70]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>70 && (*it_usersstatM).second.tot_en_tx<75){
			if(energytxM.count(75)==0){
				energytxM[75]=1; //new time -> new record in the map
			}else{
				energytxM[75]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>75 && (*it_usersstatM).second.tot_en_tx<80){
			if(energytxM.count(80)==0){
				energytxM[80]=1; //new time -> new record in the map
			}else{
				energytxM[80]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>80 && (*it_usersstatM).second.tot_en_tx<85){
			if(energytxM.count(75)==0){
				energytxM[85]=1; //new time -> new record in the map
			}else{
				energytxM[85]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>85 && (*it_usersstatM).second.tot_en_tx<90){
			if(energytxM.count(90)==0){
				energytxM[90]=1; //new time -> new record in the map
			}else{
				energytxM[90]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>90 && (*it_usersstatM).second.tot_en_tx<95){
			if(energytxM.count(95)==0){
				energytxM[95]=1; //new time -> new record in the map
			}else{
				energytxM[95]+=1; //record already inserted in the map
			}
		}else if((*it_usersstatM).second.tot_en_tx>95 && (*it_usersstatM).second.tot_en_tx<100){
			if(energytxM.count(100)==0){
				energytxM[100]=1; //new time -> new record in the map
			}else{
				energytxM[100]+=1; //record already inserted in the map
			}
		}
	}
	avg_user_num_samples=avg_user_num_samples/usersstatM[currentDay].size();
	avg_user_data=avg_user_data/usersstatM[currentDay].size();
	avg_user_energy=avg_user_energy/usersstatM[currentDay].size();
	avg_user_energy_tx=avg_user_energy_tx/usersstatM[currentDay].size();

	long double std_user_num_samples=0.0,std_user_data=0.0,std_user_energy=0.0,std_user_energy_tx=0.0;
	for(it_usersstatM= usersstatM[currentDay].begin(); it_usersstatM != usersstatM[currentDay].end(); ++it_usersstatM){

		std_user_num_samples+=pow((*it_usersstatM).second.tot_num_samples-avg_user_num_samples,2); //updating statistic for num of samples generated
		std_user_data+=pow((*it_usersstatM).second.tot_data_gen-avg_user_data,2); //updating statistic for data
		std_user_energy+=pow((*it_usersstatM).second.tot_energy-avg_user_energy,2); //updating statistic for energy
		std_user_energy_tx+=pow((*it_usersstatM).second.tot_en_tx-avg_user_energy_tx,2); //updating statistic for energy spent for transmission
	}
	std_user_num_samples=sqrt(std_user_num_samples/usersstatM[currentDay].size());
	std_user_data=sqrt(std_user_data/usersstatM[currentDay].size());
	std_user_energy=sqrt(std_user_energy/usersstatM[currentDay].size());
	std_user_energy_tx=sqrt(std_user_energy_tx/usersstatM[currentDay].size());

	long double stderr_user_num_samples=0.0,stderr_user_data=0.0,stderr_user_energy=0.0,stderr_user_energy_tx=0.0;
	stderr_user_num_samples=std_user_num_samples/sqrt(usersstatM[currentDay].size());
	stderr_user_data=std_user_data/sqrt(usersstatM[currentDay].size());
	stderr_user_energy=std_user_energy/sqrt(usersstatM[currentDay].size());
	stderr_user_energy_tx=std_user_energy_tx/sqrt(usersstatM[currentDay].size());

	usersstatM[currentDay].clear();

//	cout << endl;
//	cout << endl;
//	cout << "	-------DAY-------         "<< currentDay+1 << endl;
//	cout << "-| Printing System Statistics per Sensor" << endl;
//	cout << "-| Printing System Statistics per Sensor" << endl;
//	cout << "Accelerometer: num. samples " << acc_tot_num_samples[currentDay] << " - data " << PrintByteUnit(acc_tot_data[currentDay]) << endl;
//	cout << "Temperature: num. samples " << tem_tot_num_samples[currentDay] << " - data " << PrintByteUnit(tem_tot_data[currentDay]) << endl;
//	cout << "Pressure: num. samples " << pre_tot_num_samples[currentDay]<< " - data " << PrintByteUnit(pre_tot_data[currentDay]) << endl;
//	cout << "- - - - - - - - - - - - - -" << endl << "-| Printing per-user Statistics (" << num_users << " users)" << endl;
//	cout << "Avg number samples generated: "
//		 << avg_user_num_samples<<" - std: "
//		 << std_user_num_samples<<" - std err: "
//		 << stderr_user_num_samples << endl;
//	cout << "Avg amount of data generated: "
//		 << PrintByteUnit(avg_user_data) << " - std: "
//	     << PrintByteUnit(std_user_data) << " - std err: "
//		 << PrintByteUnit(stderr_user_data) << endl;
//	cout << "Avg amount of current drain spent for sensing: "
//		 << avg_user_energy << " uAh - std: "
//		 << std_user_energy << " uAh - std err: "
//		 << stderr_user_energy << " uAh"<<endl;
//	cout << "Avg amount of energy spent for data transmission: "
//		 << avg_user_energy_tx << " J - std: "
//		 << std_user_energy_tx << " J - std err: "
//		 << stderr_user_energy_tx << " J" << endl;




	cout << endl;
	cout << endl;
	cout << "	-------DAY-------         "<< currentDay+1 << endl;
	cout << "-| Printing System Statistics per Sensor" << endl;
	cout << "-| Printing System Statistics per Sensor" << endl;
	cout << "Accelerometer: num. samples " << acc_tot_num_samples[currentDay] << " - data " << PrintByteUnit(acc_tot_data[currentDay]) << endl;
	cout << "GPS: num. samples " << tem_tot_num_samples[currentDay] << " - data " << PrintByteUnit(tem_tot_data[currentDay]) << endl;
	cout << "Proximity: num. samples " << pre_tot_num_samples[currentDay]<< " - data " << PrintByteUnit(pre_tot_data[currentDay]) << endl;
	cout << "- - - - - - - - - - - - - -" << endl << "-| Printing per-user Statistics (" << num_users << " users)" << endl;
	cout << "Avg number samples generated: "
		 << avg_user_num_samples<<" - std: "
		 << std_user_num_samples<<" - std err: "
		 << stderr_user_num_samples << endl;
	cout << "Avg amount of data generated: "
		 << PrintByteUnit(results[(currentDay*3)]*1000) << endl;
	cout << "Avg amount of current drain spent for sensing: "
		 << results[(currentDay*3)+1] << " uAh"<<endl;
	cout << "Avg amount of current drain spent for data transmission: "
			<< results[(currentDay*3)+2] << " uAh"<<endl;

	outputStatistics  << "," <<  currentDay+1 << "," << avg_user_num_samples << "," <<PrintByteUnit(results[(currentDay*3)]*1000) << ","
					 << results[(currentDay*3)+1]<<" mAh" << "," << PrintByteUnitInMiB(acc_tot_data[currentDay]) << ","
					 << PrintByteUnitInMiB(tem_tot_data[currentDay]) << "," << PrintByteUnitInMiB(pre_tot_data[currentDay]) << ","
					 << results[(currentDay*3)+2] ;
cout << "--------------------------------------------------" << endl;

}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

/* - - - - - - - - - - - - - - - */
// CLEANING STRUCTURES
/* - - - - - - - - - - - - - - - */

smM.clear();
usersM.clear();
outputStatistics.close();

}











map <int,map<time_t,int>  > analyzeContacts(Events eventsL, Events::iterator it_eve,int radius,int numusr){

vector<vector<int> > usercon(numusr, vector<int>(40));

Events::iterator check;

int current_neigh,num_neigh,current_user;
double d;
time_t t;
it_eve=eventsL.begin();

map <int,map<time_t,int>  > usercontact;

num_neigh=0;
current_user=0;
current_neigh=0;
while(it_eve!=eventsL.end()){
	cout << (*it_eve).timestamp.tm_hour <<endl;
	check=it_eve;
	check++;
	t=mktime(&((*it_eve).timestamp));

	if(current_user!=(*it_eve).id_user){

		usercontact[current_user][t]+=num_neigh;


		current_user=(*it_eve).id_user;
		current_neigh=0;
		num_neigh=0;

	}

	while(check != eventsL.end() && (*check).timestamp.tm_min==(*it_eve).timestamp.tm_min ){
		if((*check).id_user==(*it_eve).id_user || current_neigh==(*check).id_user){
			check++;
			continue;
		}

		d=havdist((*check).loc.lat,(*check).loc.lon,(*it_eve).loc.lat,(*it_eve).loc.lon)*1000;
		if(d<radius){
			current_neigh=(*check).id_user;
			usercontact[current_neigh][t]++;
			num_neigh++;

		}
		check++;
	}

it_eve++;
}





return usercontact;

}
























