/*
 * Simulation.cc

 *
 *  Created on: 30 giu 2016
 *      Author: Giuseppe Cacciatore
 */

#include "../Headers/Simulation.h"

#include "../Headers/Utilities.h"
#include "../Headers/ReadValues.h"


const int WDdistance=50;
const int CKTthreshold=60;
const int distthr[5] = {1,10,20,30,80};
const float thr[5] = {20,16,9,2.5,1};
const float bat_threshold=100;


const int maxnumusers=9;
const int MaxDataGroup=4.944*maxnumusers;
const int NoNoGroup=0;



const float constantwB=0.33333333333;
const float constantwR= 0.33333333333;
const float constantwT=0.33333333333;
const float constantwC=0.33333333333;

//const float constantwB=0.994;
//const float constantwR= 0.003;
//const float constantwT=0.003;
//const float constantwC=0.003;


// Ricordati di sistemare GO=-1
// da sistemare caso uno solo in lista dall inizio
//aggungere proprio goai a set
int GoElection(set<Goaielement>::iterator last ,map <int,vector<Event>::iterator > eves, vector<Event>::iterator currevent){
	int idgo,idgo2;
	set<Goaielement>::iterator templast;
	while(true){
		if(last==currevent->gset.begin()){
					currevent->status="alone";
					currevent->idgo=-2;
					return -2;
				}



		if(last->id==currevent->id_user){
			currevent->status="go";
			currevent->idgo=-1;
			return -1;
		}



		if(eves[last->id]->idgo!=0){
			idgo= eves[last->id]->idgo;
		}
		else{
			templast=eves[last->id]->gset.end();
			templast--;
			idgo= GoElection(templast,eves,eves[last->id]);
		}
		if(idgo==-1){
			currevent->status="member";
			currevent->idgo=last->id;
			eves[last->id]->members++;
			return last->id;
		}
		else{
			last--;
			continue;

		}
	}



}




int updateGoai(float battery,Location loc,Samples s,float realrssi, float speed, Location lb,Location lu){
	//ofstream outp;
	//outp.open("./distance.txt");

	float wB=constantwB;
	float wR= constantwR;
	float wT=constantwT;
	int goai;
	int tex;
	float bearing;
	float distance;
	float rssi;
	float ymax=lu.lat;
	float ymin=lb.lat;

	float xmax= lu.lon* cos(( ymax * pi ) / 180);
	float xmin=lu.lon* cos(( ymax * pi ) / 180);

	float texreal;
	float texmax=((WDdistance)/1);
if(speed==0){
	texreal=texmax;
	wB=0.03;
	wR= 0.03;
	wT=0.94;
}
else{
	if(s.size()!=0){

		Samples::iterator sit=--s.end();
		float ypre=sit->pos.loc.lat;
		float xpre=sit->pos.loc.lon * cos(( ymax * pi ) / 180);

		float ynow=loc.lat;
		float xnow=loc.lon * cos(( ymax * pi ) / 180);

		float lat1 = ypre * pi / 180;
		float lat2 = ynow * pi / 180;

		float long1 = sit->pos.loc.lon * pi / 180;
		float long2 = loc.lon * pi / 180;



	/*    float lat1 = 40.7486 * pi / 180;
		float lat2 = 40.7346 * pi / 180;

		float long1 = -73.9864 * pi / 180;
		float long2 = -73.8964 * pi / 180;*/

		bearing = atan2(sin(long2-long1) * cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(long2-long1));

		bearing = bearing * 180 / pi;
		bearing = int(bearing + 360 ) % 360;

		//cout<<bearing<<"   Cord " << ypre << " "<<sit->pos.loc.lon<< " " <<ynow<< " " <<  loc.lon<< endl;
		int flaggoai=0;
		float m,q;
		float yp,xp;
		if(bearing==0 or bearing==180){
			xp=xnow;
			if(bearing==0)
				yp=ymax;


			if (bearing==180)
				yp=ymin;
			flaggoai=1;
		}
		else{

			m=(ynow-ypre)/(xnow - xpre);
			q=ypre -(m*xpre);

			if(bearing >180){
				flaggoai=2;
				xp=xmin;
				yp=m*(xp) +q;

				if(yp<ymin){
					yp=ymin;
					xp=(yp-q)/m;

				}
				if(yp>ymax){
					yp=ymax;
					xp=(yp-q)/m;

				}






			}
			else{
				flaggoai=3;
				xp=xmax;
				yp=m*(xp) +q;

				if(yp<ymin){
					yp=ymin;
					xp=(yp-q)/m;

				}
				if(yp>ymax){
					yp=ymax;
					xp=(yp-q)/m;

				}


			}




		}

		distance = 110250 * sqrt(pow((xp-xnow),2)+pow((yp-ynow),2));
		//cout<<lu.lat << " "<<lb.lat<<" flag " <<flaggoai<<"   "<<bearing<<"   distance " << int(distance)<<" Cord "<< yp << " "<<xp<< " "<<xnow << " "<<cos(( ymax * pi ) / 180)<< " " <<ynow<< " " <<  loc.lon<<  endl;

	}
	else{
		float distmin;		distance=100;
		distmin=(float)havdist(loc.lat,loc.lon,ymax,loc.lon);
		if(distmin<distance)
			distance=distmin;

		distmin=(float)havdist(loc.lat,loc.lon,ymin,loc.lon);
		if(distmin<distance)
			distance=distmin;

		distmin=(float)havdist(loc.lat,loc.lon,loc.lat,lu.lon);
		if(distmin<distance)
			distance=distmin;

		distmin=(float)havdist(loc.lat,loc.lon,loc.lat,lb.lon);
		if(distmin<distance)
			distance=distmin;
		distance=distance*1000;
	}


	texreal=(distance/speed);
}
	tex=(int)((texreal/texmax)*100);

	rssi=(realrssi/5)*100;

	goai=wB*battery +wR *rssi + wT* tex;

	//cout<<goai<< " "<<battery<<" TEX  " <<tex<<"   distance " << int(distance)<<endl;
	//outp.close();
	return goai;
}



int updateGoaiPoi(float battery,int minwait,Samples s,float realrssi, float speed, Location poi){
	//ofstream outp;
	//outp.open("./distance.txt");
	float wB=constantwB;
	float wR= constantwR;
	float wT=constantwT;

	int goai;
	int tex;
	float bearing;
	float distance;
	float rssi;
	float normwait;
//	float ypoi=poi.lat;
//	float xpoi=poi.lon* cos(( ypoi * pi ) / 180);
//
//	float texreal;
//	float texmax=(WDdistance/1);
//if(speed==0)
//	texreal=texmax;
//else{
//
//	if(s.size()!=0){
//
//		Samples::iterator sit=--s.end();
//		float ypre=sit->pos.loc.lat;
//		float xpre=sit->pos.loc.lon * cos(( ypoi * pi ) / 180);
//
//		float ynow=loc.lat;
//		float xnow=loc.lon * cos(( ypoi * pi ) / 180);
//
//		float lat1 = ypre * pi / 180;
//		float lat2 = ynow * pi / 180;
//
//		float long1 = sit->pos.loc.lon * pi / 180;
//		float long2 = loc.lon * pi / 180;
//
//
//
//	/*    float lat1 = 40.7486 * pi / 180;
//		float lat2 = 40.7346 * pi / 180;
//
//		float long1 = -73.9864 * pi / 180;
//		float long2 = -73.8964 * pi / 180;*/
//
//		bearing = atan2(sin(long2-long1) * cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(long2-long1));
//
//		bearing = bearing * 180 / pi;
//		bearing = int(bearing + 360 ) % 360;
//
//		//cout<<bearing<<"   Cord " << ypre << " "<<sit->pos.loc.lon<< " " <<ynow<< " " <<  loc.lon<< endl;
//		int flaggoai=0;
//		float yp,xp,yp2,xp2;
//
//
//
//
//		float cx=xpoi;
//		float cy=ypoi;
//		float radius = (float)WDdistance/(2*110250);
//
//		float dx, dy, A, B, C, det, t;
//
//		dx = xnow - xpre;
//		dy = ynow - ypre;
//
//		A = (dx * dx) + (dy * dy);
//		B = 2 * ((dx * (xpre - cx)) + (dy * (ypre - cy)));
//		C = ((xpre - cx) * (xpre - cx)) + ((ypre - cy) * (ypre - cy)) - (radius * radius);
//
//		det =(B * B) -( 4 * A * C);
//
//		// Two solutions.
//		t = (float)((-B + sqrt(det)) / (2 * A));
//
//		xp=xpre + t * dx;
//		yp=ypre + t * dy;
//
//		t = (float)((-B - sqrt(det)) / (2 * A));
//
//		xp2=xpre + (t * dx);
//		yp2=ypre + (t * dy);
//
//		if(bearing==0 || bearing==180){
//
//			if(bearing==0 and yp2>yp)
//				yp=yp2;
//
//
//			if(bearing==180 and yp2<yp)
//				yp=yp2;
//
//		}
//		else{
//
//
//			if(bearing<180 and xp2>xp){
//				xp=xp2;
//				yp=yp2;
//			}
//
//			if(bearing>180 and xp2<xp){
//				xp=xp2;
//				yp=yp2;
//			}
//
//		}
//
//
//
//
//
//
//
//
//
//		distance = 110250 * sqrt(pow((xp-xnow),2)+pow((yp-ynow),2));
//		cout<<A<<" "<<det<<"   distance " << int(distance)<<" Cord "<< yp << " "<<xp<< " "<<xnow << " "<<cos(( ypoi * pi ) / 180)<< " " <<ynow<< " " <<  loc.lon<<" " <<ypoi<< " " <<  poi.lon<<  endl;
//
//	}
//	else{
//		float dist;
//		dist=(float)havdist(loc.lat,loc.lon,poi.lat,poi.lon);
//		dist=dist*1000;
//		dist= WDdistance-dist;
//	}
//
//
//	texreal=(distance/speed);
//}
//	tex=(int)((texreal/texmax)*100);

	rssi=(realrssi/5)*100;
	if(minwait<0)
		minwait=0;
	normwait=(minwait/15)*100;
	goai=wB*battery +wR *rssi + wT* normwait;

	//cout<<goai<< " "<<battery<<" TEX  " <<tex<<"   distance " << int(distance)<<endl;
	//outp.close();
	return goai;
}




vector<float> simulationOperations(int num_users,Smartphones smM, Smartphones::iterator it_smM,map <time_t,vector<Event> > contacts, Users usersM, Users::iterator it_usersM,int days,Events eventsL2,Events::iterator it_eventsL2,map <int , pair<Location,Location> > grid){


	// WiFi
	double rho_tx=0.27; // WiFi power in transmission (W)
	double rho_id=3.68; // WiFi power in idle mode (W)
	double lambda_g=1000.0; // Rate of generation of packets
	double gamma_g=0.11*pow(10,-3); // Energy cost to elaborate a generated packet (J)
	int wifi_uplink_data_tx=1000000;// WiFi uplink data rate: 1 Mbps
	int datatotali=0;
	double acc_sample_frequency=10; // Hz
	double acc_sample_size=6*8;// bit
	int acc_current=450;// uA

/*	double tem_sample_frequency=50; // Hz
	double tem_sample_size=16;// bit
	int tem_current=1*tem_sample_frequency;*/// uA
	vector<float> results;
	float datatotalitotgiorni=0.0;
	float cons_sensing=0.0;
	float cons_report=0.0;

	pair<float,float> locstop;

	double prox_sample_frequency=10; // Hz
	double prox_sample_size=2*8;// bit
	int prox_current=150;// uA

	int packtotali=0;
	double gps_sample_frequency=0.1; // Hz
	double gps_sample_size=24*8;// bit
	int gps_current=22000;// uA
	float totcontribution=0;
	float quadcontribution=0;
	float batcontrib=0;
	float quadbatcontrib=0;
	float datasent=0;
	float dataexpected;



	
	// Real consumption per minute, 1- thx while sensing 2- transfer after sensing 3- random thx
	static const float arr[] = {2.22,0.917,1.18,0.561};
	// probabilistic soglia 0.25= 1.22333   0.5= 1.08
	 float pbrep_cost=1.8;
	 float threshold=0.5;
	 //soglia probabilistic
	 float reporting_cost= (2.011*11) /1200;  // costo di report per minute (mah/minute)
	 int bytes_per_minute=40; //kB per minute
	  //soglia conitnuous
	 int kbpermin=109;//kb tx per min
	 float b=1;
	 float upbound=1;
	 float lowbound=0.20;


	vector<float> cons (arr, arr + sizeof(arr) / sizeof(arr[0]) );

    int ctx=0;

    Smartphones active;
    Smartphones stopped;




    float sb;
    float cap;
    DatasentM temp_dtsM;
    DatatbdelM temp_dtbdelM;
    Samples temp_s;
    float old_battery;
    float enTot;

    vector<int> timeroute;

    float ref_lat;
	float ref_lon;
	bool pda=false;

    /* - - - - - - - - - - - - - - - - - - - */
	 // SIMULATION
	 /* - - - - - - - - - - - - - - - - - - - */


	int typeCons=readTypeCons();
    //int typeCons = 0;

	if(typeCons==4){
		typeCons=2;
		pda=true;
	}



	cout << typeCons << endl;
	ofstream outputsimdata;
	ofstream stopfile;
	ofstream activefile;
	ofstream routesfile;
	ofstream  fairfile;
	ofstream  batfairfile;
	ofstream tres;
	ofstream  membersfile;
	ofstream  wdconsfile;
	wdconsfile.open("../data/Outputs/stat/WDconsumption.txt");
	membersfile.open("../data/Outputs/stat/Members.txt");
	ofstream  memconsfile;
	memconsfile.open("../data/Outputs/stat/ConsumptionMember.txt");
	ofstream  goconsfile;
	goconsfile.open("../data/Outputs/stat/ConsumptionGo.txt");
	ofstream  ngconsfile;
	ngconsfile.open("../data/Outputs/stat/ConsumptionNg.txt");
	ofstream  rolefile1;
	rolefile1.open("../data/Outputs/r/roles1.txt",ios::app);


	ofstream  rolefile;
	rolefile.open("../data/Outputs/stat/roles.txt");

	rolefile.close();
	rolefile.open("../data/Outputs/stat/roles.txt",ios::app);

	ofstream exceedfile;
	ofstream noexceedfile;
	exceedfile.open("../data/Outputs/stat/exceed.txt");
	exceedfile.close();
	exceedfile.open("../data/Outputs/stat/exceed.txt",ios::app);
	noexceedfile.open("../data/Outputs/stat/noexceed.txt");
	noexceedfile.close();
	noexceedfile.open("../data/Outputs/stat/noexceed.txt",ios::app);
	ifstream perc;
	ifstream dur;
	ostringstream fileNameStream;                    // let this be empty
	fileNameStream << "../data/Outputs/DataFairness_" << typeCons << ".txt"; // and pass "dice_" here
	string fileName = fileNameStream.str(); 
	fairfile.open(fileName.c_str(),ios::app);
	
	ostringstream fileNameStream2;                    // let this be empty
	fileNameStream2 << "../data/Outputs/BatFairness_" << typeCons << ".txt"; // and pass "dice_" here
	string fileName2 = fileNameStream2.str(); 
	batfairfile.open(fileName2.c_str(),ios::app);
	 outputsimdata.open("../data/Outputs/SimulationData.txt");
	 stopfile.open("../data/Outputs/stat/Groups.txt");
	 activefile.open("../data/Outputs/Active.txt");
	 routesfile.open("../results/Routes.txt");
	 tres.open("../data/Outputs/tres.txt");
	 perc.open("../data/Inputs/perc_connections");
	 dur.open("../data/Inputs/durations");

	 outputsimdata << "/Day/-" << "/Hour/-" << "/Minute/-" << "/ID-User/-" << "/Lat/-"
			 	   << "/Long/- "<< "/BatteryLevel/-" << "/SensorDataInformation/-" << endl;
	 activefile << "/ID-User/-" <<  "/DataGenerated/- "<< "/EnergyConsumption/-"<< "/BatteryConsumption/-"<< "/BatteryLevel/- Minutes of Contribution -/ Timestamp -/ last position" << endl;

	 int currentDay=0;
	 vector<float> percentages;
	 vector<int> duration_hour;

	 bool traces =false;
	 float volt=3.7;
	 int packold=0;


	vector<pair<float, float> > route;
	vector<bool> generate;
	bool gen;
	bool flagstart;
	int numgeneration;
	int duration;
	int ref_user=-1;

	int counteruser=0;
	Smartphone sm(0,0,0,0,"",temp_dtsM,temp_dtbdelM,temp_s,0,route,generate,true,true,0,0,0,0,timeroute,0,0);


	 cout << "**Start simulation**" << endl;
	 if(typeCons==3){

		 float tmp;

		 while(perc >> tmp){
			 percentages.push_back(tmp);
			 //cout<< tmp<<endl;
		 }

		 while(dur >> tmp){
				duration_hour.push_back(tmp);
				//cout<< tmp<<endl;
			 }




	 }

 if(traces==false)
	 kbpermin=((60*wifi_uplink_data_tx)/8); //Bytes

 dataexpected=555.0125*num_users;





int numchanges=0;
int numsamego=0;



vector<Event>::iterator it3;

map<int,set<int> > members;

map <int,vector<Event>::iterator > eves;
map<int,int> groupduration;
map<int,int> groupmems;
map <int,int > gomap;
map <int,int >::iterator g;
bool newelection=false;
map<int,float> dataPerGroup;
map<int,float> consumptionGo;
map<int,int> mapmember;
map<int,float> dataGoMin;
float dist;


int numtot=0,numalone=0,nummember=0,numgos=0;



float ex_go=0,ex_mem=0,ex_ng=0,ex_nummem=0;
int ex_tot=0;

float no_go=0,no_mem=0,no_ng=0,no_nummem=0;
int no_tot=0;

for (map <time_t,vector<Event> >::iterator it = contacts.begin(); it != contacts.end(); ++it) {

	vector<Event> evl=it->second;


	members.clear();
	eves.clear();
	mapmember.clear();
	for (vector<Event>::iterator it2 = evl.begin(); it2 != evl.end(); ++it2){

		members[it2->quad].insert(it2->id_user);
		eves.insert(pair <int,vector<Event>::iterator >(it2->id_user,it2));
		//cout<<it2->quad <<"   "<<grid[it2->quad].first.lat<<"   "<<grid[it2->quad].second.lat<<" "<< (*it2).loc.lat<<endl;
		//if((it2->id_user)==1)
			//cout<<"Trovato0"<<it2->quad<<"  "<<it2->loc.lat<<"   "<<it2->loc.lon<<" "<< it2->timestamp.tm_min<<endl;
	}


	for(map <int,set<int> >::iterator mit = members.begin(); mit != members.end(); ++mit){
		newelection=false;
		g=gomap.find(mit->first);

		if(g !=gomap.end()){


			set<int>::iterator check= mit->second.find(g->second);
			if( check !=mit->second.end()){
				map <int,vector<Event>::iterator >::iterator goev=eves.find((*check));
				gomap[mit->first]=goev->second->id_user;
				goev->second->status="go";
				goev->second->idgo=-1;
				goev->second->members=mit->second.size();
				mapmember[goev->second->quad]=mit->second.size();
				numsamego++;
				groupduration[mit->first]++;


				//cout<<"members "<<goev->second->members<<endl;

				for(set<int>::iterator sit = mit->second.begin(); sit != mit->second.end(); ++sit){
					//if((*sit)==1)
						//cout<<"Trovato"<<endl;
					//it_smM=smM.find((*sit));
					if((*sit)==goev->second->id_user)
						continue;

					map <int,vector<Event>::iterator >::iterator tev=eves.find((*sit));

					dist=havdist(goev->second->loc.lat,goev->second->loc.lon,tev->second->loc.lat,tev->second->loc.lon);
					dist=dist*1000;
					//cout<<dist<<endl;
					tev->second->distgo=dist;
					tev->second->status="member";
					tev->second->idgo=mit->first;
					//smM[(*sit)].goai=15;
					//cout<<mit->first<<"  "<<((*tev).second.quad) <<"   "<<grid[mit->first].first.lat<<"   "<<grid[mit->first].second.lat<<" "<< (*tev).second.loc.lat<<endl;

				}

				continue;
			}
			else{
				gomap.erase(mit->first);
				newelection=true;
			}



		}
		else{
			newelection=true;
		}



		if(newelection==true){
			if(dataPerGroup.find(mit->first)!=dataPerGroup.end())
				if(groupduration[mit->first]!=0)
					stopfile<<"Durata "<<groupduration[mit->first]<<" Data "<<dataPerGroup[mit->first]<<" membsAVG "<< groupmems[mit->first]<<endl;

			groupduration[mit->first]=1;
			groupmems[mit->first]=0;
			dataPerGroup[mit->first]=0;

			numchanges++;
			int goaimax=0;
			int idmaxg=-1;
			for(set<int>::iterator sit = mit->second.begin(); sit != mit->second.end(); ++sit){
				//it_smM=smM.find((*sit));
				map <int,vector<Event>::iterator >::iterator tev=eves.find((*sit));
				tev->second->status="member";
				tev->second->idgo=mit->first;
				//smM[(*sit)].goai=15;
				//cout<<mit->first<<"  "<<((*tev).second.quad) <<"   "<<grid[mit->first].first.lat<<"   "<<grid[mit->first].second.lat<<" "<< (*tev).second.loc.lat<<endl;
				int goai_temp= updateGoai(smM[(*sit)].battery,tev->second->loc,smM[(*sit)].smp,tev->second->rssi,tev->second->speed,grid[mit->first].first,grid[mit->first].second);
				smM[(*sit)].goai=goai_temp;
				if(goai_temp>goaimax){
					goaimax=goai_temp;
					idmaxg=(*sit);
				}

				//if(tev->second->idgo==-2)
					//cout<<"AALRMEEEEEEEEEEE 10"<<endl;

			}
			if(idmaxg!=-1){

				map <int,vector<Event>::iterator >::iterator tev2=eves.find(idmaxg);
				gomap[mit->first]=tev2->second->id_user;
				tev2->second->status="go";
				tev2->second->idgo=-1;
				tev2->second->members=mit->second.size();
				mapmember[tev2->second->quad]=mit->second.size();
				//cout<<"members "<<tev2->second->members<<endl;


				for(set<int>::iterator sit2 = mit->second.begin(); sit2 != mit->second.end(); ++sit2){
					map <int,vector<Event>::iterator >::iterator evit2=eves.find((*sit2));
					if(evit2->second->idgo>0){
						float distg;

						distg=havdist(tev2->second->loc.lat,tev2->second->loc.lon,evit2->second->loc.lat,evit2->second->loc.lon);
						distg=distg*1000;
						//cout<<distg<<" "<<tev2->second->id_user<<" "<<evit2->second->id_user<<" "<<evit2->second->timestamp.tm_min<<endl;
						evit2->second->distgo=distg;

					}


				}



			}







		}


	}







		sort (evl.begin(), evl.end(), idgocomp);

		consumptionGo.clear();
		dataGoMin.clear();

		for (vector<Event>::iterator it_eventsL = evl.begin(); it_eventsL != evl.end(); ++it_eventsL){



			numtot++;



			//if((it_eventsL->id_user)==1)
						//cout<<"Trovato2"<<endl;



			if(it_eventsL->idgo>0 and dataGoMin[it_eventsL->idgo]> MaxDataGroup){

				int idgotmp=gomap[it_eventsL->idgo];
				//cout<<"Taglio "<<it_eventsL->id_user<<" "<<idgotmp<<endl;

				mapmember[it_eventsL->idgo]--;
				if(mapmember[it_eventsL->idgo]<0){
					//cout<<idgotmp<<" "<<it_eventsL->idgo<<" "<<mapmember[it_eventsL->idgo]<<" "<<dataGoMin[it_eventsL->idgo]<<" "<<it_eventsL->id_user<<endl;
					exit(-1);
				}
				it_eventsL->idgo=-2;
			}



			if(it_eventsL->idgo==-2){
				numalone++;
			}


			if(it_eventsL->idgo==-1 ){
				numgos++;


				if(mapmember[it_eventsL->quad]!=it_eventsL->members){
					it_eventsL->members=mapmember[it_eventsL->quad];
					//cout<<"memeber tagliato "<<it_eventsL->id_user<<endl;

				}

				membersfile<< it_eventsL->id_user <<" "<<mapmember[it_eventsL->quad]<<endl;
				groupmems[it_eventsL->quad]=groupmems[it_eventsL->quad] + (mapmember[it_eventsL->quad] -1);
				//cout<<"GO "<<it_eventsL->id_user<<" data "<<dataPerGroup[it_eventsL->quad]<<" quad "<<it_eventsL->quad<<" durata " <<groupduration[it_eventsL->quad] <<" members "<< it_eventsL->members<<" min "<<it_eventsL->timestamp.tm_min<<" cons "<<consumptionGo[it_eventsL->quad]<<" "<<dataGoMin[it_eventsL->quad]<<endl;

			}





		 if(ref_user!=(*it_eventsL).id_user){ //da mantenere
			 if(ref_user==-1){
				 ref_user=1;
			 }

			else{





				smM.erase(ref_user);
				Smartphone temp_str(ref_user,sm.startBattery,old_battery,cap,"Outdoor",temp_dtbdelM,temp_dtsM,temp_s,sm.stop,route,generate,gen,flagstart,numgeneration,duration,sm.minwalk,sm.goai,timeroute,sm.idgo,sm.members);
				smM.insert(pair <int, Smartphone >(ref_user,temp_str) );

				temp_s.clear();
				temp_dtsM.clear();
				temp_dtbdelM.clear();
				locstop=make_pair(0,0);

			}
		 ref_user=(*it_eventsL).id_user;

		 it_smM=smM.find(ref_user);
		 it_usersM=usersM.find(ref_user);
		 sm=(*it_smM).second;
		 temp_dtbdelM=sm.dtbd;
		 temp_dtsM=sm.dts;

		 temp_s=sm.smp;
		 old_battery=sm.battery;
		 cap=sm.capacity;
		 generate=sm.generate;
		 gen=sm.gen;
		 duration=sm.duration;
		 numgeneration=sm.numgeneration;
		 flagstart=sm.flagstart;
		 route=sm.route;
		 timeroute=sm.timeroute;

		 int ref_smartphone=sm.id_user;
		 string ref_context=sm.context;

		 }


		 ref_lat=(*it_eventsL).loc.lat;
		 ref_lon=(*it_eventsL).loc.lon;
		 float ref_alt=(*it_eventsL).loc.alt;









		 tm prev_timestamp;
		 Samples::iterator it_tmpsample;

		 it_tmpsample= --temp_s.end();





		 if(currentDay!=(*it_eventsL).timestamp.tm_wday){
		 				 int userusati=ref_user-counteruser;
		 				 results.push_back(datatotali/userusati);
		 				 results.push_back(cons_sensing/userusati);
		 				 results.push_back(cons_report/userusati);
		 				 cons_report=0;
		 				 cons_sensing=0;
		 				 datatotali=0;

		 				 counteruser=ref_user;
		 			 }
		 //inserted to avoid for
		 currentDay=(*it_eventsL).timestamp.tm_wday;

		 prev_timestamp=(*it_tmpsample).pos.timestamp;
		 //serve???
		 //prev_timestamp.tm_wday = currentDay;

		 // if sample list empty previous timestamp = actual event ts
		 if(temp_s.size() == 0){
			 prev_timestamp=(*it_eventsL).timestamp;
		 }

		 // edited first event of the day has to be 0 as difference of minutes between samples
		 int time_between_samples = 0;
		 if((*it_eventsL).timestamp.tm_wday == prev_timestamp.tm_wday){
			 if((*it_eventsL).timestamp.tm_hour == prev_timestamp.tm_hour){
				 time_between_samples = abs((*it_eventsL).timestamp.tm_min-prev_timestamp.tm_min);
			 }


			 if((*it_eventsL).timestamp.tm_hour == prev_timestamp.tm_hour+1){
				 time_between_samples = ((*it_eventsL).timestamp.tm_min+(60 - prev_timestamp.tm_min));
			 }


			 //non dovrebbe entrarci mai?
			 if((*it_eventsL).timestamp.tm_hour > prev_timestamp.tm_hour+1){
				 int hours_counter2 = (*it_eventsL).timestamp.tm_hour - prev_timestamp.tm_hour ;
				 time_between_samples = hours_counter2*60+abs(((*it_eventsL).timestamp.tm_min-prev_timestamp.tm_min));
			 }
		 }

		 /*** GENERATION OF NEW SAMPLES ***/
		 if(flagstart==true){
			 time_between_samples=0;
		 }
		 else{
			 time_between_samples=1;
		 }


		 sm.minwalk=sm.minwalk + time_between_samples;


		 // Always on sensors: consider sampling frequency, take time between previous timestamp and current
		 // and generate a number of samples accordingly

		 float temp_value=fRand(1.0,5.0);
		 float temp_size=fRand(500.0, 2000.0); // Kilobyte
		 string temp_type="GPS";

		 // GPS
		 Location temp_loc(ref_lat,ref_lon,ref_alt);
		 Position temp_p(temp_loc,(*it_eventsL).timestamp);
		 Sample temp_sample(temp_type,temp_value,temp_size,false,temp_p);
		 temp_s.push_back(temp_sample);


			 float tau_tx=lambda_g*(0.000192+((28.0+1500.0)/wifi_uplink_data_tx));// 192 us: PLCP time + (Header+Payload)/data rate
			 float wifi_power=rho_id+rho_tx*tau_tx+lambda_g*gamma_g;// power to tx one packet

		// * * ACCELLEROMETER
			 int acc_num_samples_to_generate=acc_sample_frequency*60*time_between_samples;
			 int acc_data_size=(acc_num_samples_to_generate*acc_sample_size)/8;// to have Bytes
			 int acc_energy_sampling=acc_current*(60*time_between_samples);// these are uAs (remember: 3600 uAs= 1uAh)
			 float acc_energy_sampling_hour=acc_energy_sampling/3600000.00;// mAh

		int acc_packets=(int)acc_data_size/1500;
		float acc_tx_time=(acc_data_size*8.0)/wifi_uplink_data_tx;// in seconds (bit / bps)

		float acc_wifi_energy=wifi_power*acc_packets; // Joules (Watt*seconds)
		float acc_wifi_consumption=( acc_wifi_energy/( 1000*3.6 ) )*( 1000/volt ); // mah

		outputsimdata << currentDay << " "<<(*it_eventsL).timestamp.tm_hour << " "
					  << (*it_eventsL).timestamp.tm_min << " "
					  << sm.id_user << " "
					  << ref_lat << " "// position latitude
					  << ref_lon << " "// position longitude
					  << sm.battery << " "
					  << "Accelerometer" << " "
					  << acc_num_samples_to_generate << " "
					  << acc_data_size << " "// in Bytes
					  << acc_packets << " "// number of packets
					  << acc_tx_time << " "// uplink transmission time in seconds
					  << acc_energy_sampling_hour << " "// in uAh
					  << acc_wifi_energy//
					  << endl;

		// * * TEMPERATURE
		int gps_num_samples_to_generate=gps_sample_frequency*60*time_between_samples;
		int gps_data_size=(gps_num_samples_to_generate*gps_sample_size)/8;// to have Bytes
		int gps_energy_sampling=gps_current*(60*time_between_samples)*gps_sample_frequency;// these are uAs (remember: 3600 uAs= 1uAh)
		float gps_energy_sampling_hour=gps_energy_sampling/3600000.00;// uAh

		int gps_packets=(int)gps_data_size/1500;
		float gps_tx_time=(gps_data_size*8.0)/wifi_uplink_data_tx;// in seconds (Byte / Bps)
		float gps_wifi_energy=wifi_power*gps_packets; // Joules (Watt*seconds)

		float gps_wifi_consumption=( gps_wifi_energy/( 1000*3.6 ) )*( 1000/volt );

		outputsimdata << currentDay << " " << (*it_eventsL).timestamp.tm_hour << " "
					  << (*it_eventsL).timestamp.tm_min << " "
					  << sm.id_user << " "
					  << ref_lat << " "// position latitude
					  << ref_lon << " "// position longitude
					  << sm.battery << " "
					  << "GPS" << " "
					  << gps_num_samples_to_generate << " "
					  << gps_data_size << " "// in Bytes
					  << gps_packets << " "
					  << gps_tx_time << " "// uplink transmission time in seconds
					  << gps_energy_sampling_hour << " "// in mAh
					  << gps_wifi_energy
					  << endl;

		// * * PRESSURE
		int prox_num_samples_to_generate=prox_sample_frequency*60*time_between_samples;
		int prox_data_size=(prox_num_samples_to_generate*prox_sample_size)/8;// to have Bytes
		int prox_energy_sampling=prox_current*(60*time_between_samples);// these are uAs (remember: 3600 uAs= 1uAh)
		float prox_energy_sampling_hour=prox_energy_sampling/3600000.00;// mAh

		int prox_packets=(int)prox_data_size/1500;
		float prox_tx_time=(prox_data_size*8.0)/wifi_uplink_data_tx;// in seconds (Byte / Bps)
		float prox_wifi_energy=wifi_power*prox_packets; // Joules (Watt*seconds)
		float prox_wifi_consumption=( prox_wifi_energy/( 1000*3.6 ) )*( 1000/volt );

		outputsimdata << currentDay << " " << (*it_eventsL).timestamp.tm_hour << " "
					  << (*it_eventsL).timestamp.tm_min << " "
					  << sm.id_user << " "
					  << ref_lat << " "// position latitude
					  << ref_lon << " "// position longitude
					  << sm.battery << " "
					  << "Proximity" << " "
					  << prox_num_samples_to_generate << " "
					  << prox_data_size << " "// in Bytes
					  << prox_packets << " "
					  << prox_tx_time << " "//uplink transmission time in seconds
					  << prox_energy_sampling_hour << " "// in mAh
					  << prox_wifi_energy
					  << endl;


		 temp_dtbdelM["Accelerometer"]+=acc_data_size; // number of Bytes
		 temp_dtbdelM["GPS"]+=1.0;
		 float tot_data=0;

		 float cons_sens=0;
		 float cons_wifi=0;
		 float tot_wifi_energy;
		 int packs;
		 if(sm.stop==0){
			 tot_data=acc_data_size+prox_data_size+gps_data_size;
			 temp_dtbdelM["data"]+=tot_data;
			 cons_sens=prox_energy_sampling_hour+gps_energy_sampling_hour+acc_energy_sampling_hour;
			 temp_dtbdelM["consumption-data"]+=cons_sens;

			 packs=(int)tot_data/1500;
			 packs+=1;
			 if(typeCons==0)
				 packtotali +=packs;
			 tot_wifi_energy=wifi_power*packs; // Joules (Watt*seconds)
			 cons_wifi=( tot_wifi_energy/( 1000*3.6 ) )*( 1000/volt );
			 temp_dtbdelM["consumption-wifi"]+=cons_wifi;
			 temp_dtbdelM["Energy"]+=cons_sens;
		}


			 enTot= cons[typeCons]*temp_dtbdelM["Minutes"];

			 if(traces==false){
				 enTot=temp_dtbdelM["consumption-data"];
				 if(typeCons==0){
					 enTot+=temp_dtbdelM["consumption-wifi"];
					 temp_dtbdelM["Energy"]+=cons_wifi;
				 }
					 }

			 float enmax= (cap/100)*bat_threshold;




			 if(pda==true){
				 float si=datasent/dataexpected;
				 if(b>0 ){
					 if(si>upbound){
						 if(b>1)
							 b=b-1;

						 else
							 b=b/(1+b);

					 }
				 }
				if(b<50 ){
					 if(si<lowbound){
						 if(b<1)
							 b=b/(1-b);
						 else
							 b=b+1;
					 }
				 }
				 b=0.5;
				 threshold=1-pow(si,b);
				 tres<< threshold<<" "<<datasent<<" "<<b<<" "<<si<<" "<<pow(si,b) <<endl;
			 }





			 if(typeCons==2){
				 float prob=(float(rand()) /float(RAND_MAX));
				 if(threshold<prob)
					 gen=false;
				 else
					 gen=true;

			 }

			 if(typeCons==3 && duration==0){
				 float prob=(float(rand()) /float(RAND_MAX));
				 if(percentages[((*it_eventsL).timestamp.tm_hour)]<prob)
					 gen=false;
				 else{
					 duration=duration_hour[((*it_eventsL).timestamp.tm_hour)];
					 gen=true;
				 }






			 }

			 if(duration>0)
				 duration-=time_between_samples;

			 if(typeCons==1)
				 gen=true;

			 if(typeCons==0){

				 if(enTot>=enmax && sm.stop==0){

					 sm.stop=1;


					 sb= sm.startBattery;
	/*					 if(typeCons==1){
						 ctx=temp_dtbdelM["Minutes"] * reporting_cost;

						 enTot+=ctx;



					 }*/

					old_battery=sm.battery;
					float rem= (cap/100)*old_battery;
					rem = rem - enTot;

					old_battery= (rem/cap)*100;
					locstop=make_pair(ref_lon,ref_lat);

					//stopfile << ref_user <<" " << temp_dtbdelM["Total"] <<" "<< enTot <<" "<< sb-old_battery <<" "<<old_battery << " "<<(*(it_eventsL)).loc.lon<<" "<<(*(it_eventsL)).loc.lat<<endl;
					 //cout<< "Stopped"<<" "<<ref_user<<endl;
					gen=false;
				 }
				 if(sm.stop==0)
					 temp_dtbdelM["Minutes"]+=time_between_samples;
			 }


				float temp_data=0.0;
				 if(gen || typeCons!=0){
					 if(traces==true)
					 if(typeCons!=0)
					 temp_dtbdelM["Minutes"]+=time_between_samples;

					 temp_dtbdelM["Total"]+=(time_between_samples * bytes_per_minute);
					 temp_data=tot_data;
					 if((typeCons==2 || typeCons==3) && gen ){
						 if(traces==true){
							 if(temp_dtbdelM["Total"]<kbpermin){
								 temp_dtsM["Total"]+=temp_dtbdelM["Total"];
								 datasent+=temp_dtbdelM["Total"];
								 temp_dtbdelM["Total"]=0;

							 }
							 else{
								 temp_dtsM["Total"]+=kbpermin;
								 temp_dtbdelM["Total"]-=kbpermin;
								 datasent+=kbpermin;
							 }
						 }
						 else{
							 temp_dtbdelM["Minutes"]+=time_between_samples;

							 if(temp_dtbdelM["data"]<kbpermin){

								 temp_dtsM["Total"]+=temp_dtbdelM["data"];
								 temp_data=temp_dtbdelM["data"];
								 int tot_packets=(int)temp_dtbdelM["data"]/1500; //Bytes/dimension of one packet
								 tot_packets+=1;
								 float wifitemp=((tot_packets*wifi_power)/( 1000*3.6 ) )*( 1000/volt);
								 temp_dtbdelM["Energy"]+=wifitemp;
								 packtotali+=tot_packets;
								 temp_dtbdelM["data"]=0;


							 }

							 else{

								 temp_dtsM["Total"]+=kbpermin;
								 temp_dtbdelM["data"]-=kbpermin;
								 temp_data=kbpermin;
								 int tot_packets=(int)kbpermin/1500;//Bytes/dimension of one packet
								 tot_packets+=1;
								 packtotali+=tot_packets;
								 temp_dtbdelM["Energy"]+=((tot_packets*wifi_power)/( 1000*3.6 ) )*( 1000/volt);

							 }



						 }



					 }
				 }
	//			 if(ref_user==42)
	//				 cout<< ref_lon<<" "<<ref_lat<<endl;
		 route.push_back(make_pair(ref_lon,ref_lat));
		 timeroute.push_back((*it_eventsL).timestamp.tm_min);

		 generate.push_back(gen);
		 if(gen==true && flagstart==false)
			 numgeneration+=1;

		float datiTemp;
		if(traces==false){

			datiTemp= temp_data/1000;    // from Byte to KB
			//enTot=temp_dtbdelM["consumption-wifi"]+ temp_dtbdelM["consumption-data"];
			//cons_sensing+=temp_dtbdelM["consumption-data"];
			//cons_report+=temp_dtbdelM["consumption-wifi"];


			if(it_eventsL->idgo!=-2)
				dataPerGroup[it_eventsL->quad]+=datiTemp;



		}



		 if(flagstart)
			 flagstart=false;

		 float upwd_perbit,uplte_perbit,downwd_perbit,thrwd,thrlte;
		 thrlte=it_eventsL->rssi;

		 //cout<< uplte_perbit <<" "<< upwd_perbit<<" " <<tot_data*8*downwd_perbit<<" "<< tot_wifi_energy <<endl;


		 if(it_eventsL->idgo>0){


			 float dt=it_eventsL->distgo;

			 for(int ij=0;ij<5;ij++){
				 if(dt<distthr[ij]){
					 thrwd=thr[ij];
					 break;
				 }
			 }

			upwd_perbit = (283.17* thrwd + 132.86)/(thrwd*wifi_uplink_data_tx*1000);


			//uplte_perbit = (438.39* thrwd + 1288.04)/(thrlte*wifi_uplink_data_tx*1000);

			downwd_perbit= (137.01* thrwd + 132.86)/(thrwd*wifi_uplink_data_tx*1000);


			consumptionGo[it_eventsL->idgo]= consumptionGo[it_eventsL->idgo] + (1000*8*datiTemp*downwd_perbit);

			dataGoMin[it_eventsL->idgo]=  dataGoMin[it_eventsL->idgo] + datiTemp;

			temp_dtbdelM["wdcons"]+= 1000*8*datiTemp*upwd_perbit;


			//cout<<"member "<<it_eventsL->id_user <<"  GO "<<gomap[it_eventsL->idgo]<<" quad "<<it_eventsL->quad<<" data "<< dataPerGroup[it_eventsL->idgo]<<" min "<<it_eventsL->timestamp.tm_min<<" datitemp "<< datiTemp <<" membs "<< mapmember[it_eventsL->idgo] <<" datimin "<<dataGoMin[it_eventsL->idgo]<<endl;
			nummember++;

			memconsfile<<1000*8*datiTemp*upwd_perbit<<endl;
			temp_dtbdelM["member"]+=1;


		 }



			if(it_eventsL->idgo==-1){
				    uplte_perbit = (438.39* thrlte + 1288.04)/(thrlte*wifi_uplink_data_tx*1000);

				    temp_dtbdelM["wdcons"]+=consumptionGo[it_eventsL->quad];
				    temp_dtbdelM["wdcons"]+=uplte_perbit*1000*8*(datiTemp + dataGoMin[it_eventsL->quad]);
				    goconsfile<<consumptionGo[it_eventsL->quad]+(uplte_perbit*1000*8*(datiTemp + dataGoMin[it_eventsL->quad]))<<endl;
					temp_dtbdelM["go"]+=1;
					temp_dtbdelM["nummember"]+=it_eventsL->members;
			}

			if(it_eventsL->idgo==-2){
					uplte_perbit = (438.39* thrlte + 1288.04)/(thrlte*wifi_uplink_data_tx*1000);
					temp_dtbdelM["wdcons"]+=NoNoGroup*uplte_perbit*(1000*8*datiTemp);
					ngconsfile<<uplte_perbit*(1000*8*datiTemp)<<endl;
					temp_dtbdelM["alone"]+=1;

			}

		 if(sm.minwalk==(*it_usersM).second.minutes){





			 enTot=0;

			 float dati;
			 sb=sm.startBattery;

			 if(typeCons==0 || typeCons==1)
				 dati=temp_dtbdelM["Total"];

			 if(typeCons==2 || typeCons==3)
				 dati=temp_dtsM["Total"];


			 if(typeCons==1 ){

				ctx=dati * reporting_cost;
				enTot+=ctx;
			}

			if(typeCons==3 || pda==true ){

				ctx=numgeneration * pbrep_cost;
				enTot+=ctx;
				enTot+= cons[3]*(temp_dtbdelM["Minutes"]-numgeneration);
				cons_sensing+=cons[3]*(temp_dtbdelM["Minutes"]-numgeneration);
				cons_report+=ctx;
			}
			else{
				enTot+= cons[typeCons]*temp_dtbdelM["Minutes"];
				cons_sensing+=cons[3]*temp_dtbdelM["Minutes"];
				cons_report+=enTot-cons[3]*temp_dtbdelM["Minutes"];
									}

			if(traces==false){

				dati= temp_dtbdelM["data"]/1000;    // from Byte to KB
				enTot=temp_dtbdelM["consumption-wifi"]+ temp_dtbdelM["consumption-data"];
				cons_sensing+=temp_dtbdelM["consumption-data"];
				cons_report+=temp_dtbdelM["consumption-wifi"];

				if(typeCons==2 || typeCons==3){
					dati=temp_dtsM["Total"]/1000;
					enTot=temp_dtbdelM["Energy"];
					cons_sensing+=temp_dtbdelM["consumption-data"];
					cons_report+=enTot-temp_dtbdelM["consumption-data"];

				}
			}



			float totalwdconsumption=0.0,nogroup_consumption,thrlte;


			thrlte=it_eventsL->rssi;
			uplte_perbit = (438.39* thrlte + 1288.04)/(thrlte*wifi_uplink_data_tx*1000);
			totalwdconsumption=temp_dtbdelM["wdcons"];


			nogroup_consumption=uplte_perbit*1000*8*dati;

			wdconsfile<<it_eventsL->id_user<<" "<<nogroup_consumption<<" "<<totalwdconsumption<<endl;


			if(totalwdconsumption>2.7){
				ex_go+=(temp_dtbdelM["go"]/sm.minwalk)*100;
				ex_ng+=(temp_dtbdelM["alone"]/sm.minwalk)*100;
				ex_mem+=(temp_dtbdelM["member"]/sm.minwalk)*100;
				ex_nummem+=temp_dtbdelM["nummember"]/temp_dtbdelM["go"];
				ex_tot+=1;

			}
					//exceedfile<<(temp_dtbdelM["alone"]/sm.minwalk)*100<<" "<<(temp_dtbdelM["go"]/sm.minwalk)*100<<" "<<(temp_dtbdelM["member"]/sm.minwalk)*100<<" "<<temp_dtbdelM["nummember"]/temp_dtbdelM["go"]<<endl;
			else{


				no_go+=(temp_dtbdelM["go"]/sm.minwalk)*100;
				no_ng+=(temp_dtbdelM["alone"]/sm.minwalk)*100;
				no_mem+=(temp_dtbdelM["member"]/sm.minwalk)*100;
				if(temp_dtbdelM["go"]!=0)
					no_nummem+=temp_dtbdelM["nummember"]/temp_dtbdelM["go"];
				no_tot+=1;

			}
				//noexceedfile<<(temp_dtbdelM["alone"]/sm.minwalk)*100<<" "<<(temp_dtbdelM["go"]/sm.minwalk)*100<<" "<<(temp_dtbdelM["member"]/sm.minwalk)*100<<" "<<temp_dtbdelM["nummember"]/temp_dtbdelM["go"]<<endl;

			if(it_eventsL->idgo==-1){
				stopfile<<"Durata "<<groupduration[it_eventsL->quad]<<" Data "<<dataPerGroup[it_eventsL->quad]<<" membsAVG "<<groupmems[it_eventsL->quad]<<endl;
				groupduration[it_eventsL->quad]=0;
				groupmems[it_eventsL->quad]=0;
				dataPerGroup[it_eventsL->quad]=0;
			}

			old_battery=sm.battery;
			float rem= (cap/100)*old_battery;
			rem = rem - enTot;

			old_battery= (rem/cap)*100;
			smM.erase(ref_user);
			Smartphone temp_str(ref_user,sb,old_battery,cap,"Outdoor",temp_dtbdelM,temp_dtsM,temp_s,sm.stop,route,generate,gen,flagstart,numgeneration,duration,sm.minwalk,sm.goai,timeroute,sm.idgo,sm.members);
			smM.insert(pair <int, Smartphone >(ref_user,temp_str) );

			if(locstop.first==0){
				locstop=make_pair(ref_lon,ref_lat);
			}
			datatotali+=dati;
			datatotalitotgiorni+=dati;

			float mintotali=(*it_usersM).second.minutes;
			if(mintotali!=0){
			float contrib=numgeneration/mintotali;
			float batteryrel= enTot/(rem + enTot);
			batcontrib+=batteryrel;
			quadbatcontrib+=batteryrel*batteryrel;
			totcontribution +=contrib;
			quadcontribution+=contrib*contrib;
			}
			activefile << sm.id_user <<" "<< dati<<" "<<enTot <<" "<< sb-old_battery <<" "<<old_battery << " "<<numgeneration << " "<< (*(it_eventsL)).timestamp.tm_hour<<","<<(*(it_eventsL)).timestamp.tm_min<< " "<<locstop.first<<" "<<locstop.second <<" "<< temp_dtbdelM["consumption-wifi"]<< " "<<(*it_usersM).second.minutes << endl;

			int k=0;
			//if(ref_user==23 || ref_user==85||ref_user==119 ||ref_user==53 ||ref_user==106 )
			float datigenpermin=dati/numgeneration;
			float dg=0.0;
			if((*(it_eventsL)).timestamp.tm_hour==12 && (*(it_eventsL)).timestamp.tm_wday==0)
			for(int i=0;i<(route.size()-1);i++){
				dg=0.0;
				if(generate[i+1]==true){
					dg=datigenpermin;
					routesfile <<timeroute[i]<<"	"<<route[i].first<<" "<<route[i].second<< "	"<< generate[i+1] <<"	"<<dg<< endl;
				}
			}


		 }


	}


 }

		rolefile1<<"1 "<<((float)numalone/(float)numtot)*100<<" "<<((float)numgos/(float)numtot)*100<<" "<<((float)nummember/(float)numtot)*100<<endl;
		rolefile<<"1 "<<((float)numalone/(float)numtot)*100<<" "<<((float)numgos/(float)numtot)*100<<" "<<((float)nummember/(float)numtot)*100<<endl;
		cout <<"DATA Generated   "<<datatotali<<endl;
		 fairfile <<"tot Contribution  "<<totcontribution<<" quad contrib  "<<quadcontribution<<" Fairness  "<<((totcontribution*totcontribution)/quadcontribution)/num_users<<endl;
		
		 batfairfile <<"BATTERY CONTRIBUTION  "<<batcontrib<<" quad contrib  "<<quadbatcontrib<<" Fairness  "<<((batcontrib*batcontrib)/quadbatcontrib)/num_users<<endl;

		 cout <<"cambiamenti  "<<numchanges<<" stesso go  "<<numsamego<<endl;

		 exceedfile<<"1 "<<ex_ng/ex_tot<<" "<<ex_mem/ex_tot<<" "<<ex_go/ex_tot<<" "<<ex_nummem/ex_tot<<endl;
		 noexceedfile<<"1 "<<no_ng/no_tot<<" "<<no_mem/no_tot<<" "<<no_go/no_tot<<" "<<no_nummem/no_tot<<endl;
		 int userusati2=ref_user-counteruser;
		 results.push_back(datatotali/userusati2);
		 results.push_back(cons_sensing/userusati2);
		 results.push_back(cons_report/userusati2);
		 cons_report=0;
		 cons_sensing=0;
		 datatotali=0;

		counteruser=ref_user;

		perc.close();
	    outputsimdata.close();
	    stopfile.close();
	    activefile.close();

	    return results;




}


int ClosestPoi(Location p,map <int, pair<Location,Location> > grid, vector<Location> vecpoi,int numpoipast){
	double disttemp=0,distmin=1000;
	int idpoi=-1;
	if(numpoipast!=0)
	for (vector<Location>::iterator it = vecpoi.begin(); it != vecpoi.end(); ++it){
		if(((int)it->alt)==numpoipast){
			disttemp=havdist(p.lat,p.lon,it->lat,it->lon);
			disttemp=disttemp*1000;
			if(disttemp < (WDdistance/2))
				return numpoipast;
			else
				break;
		}

	}



	for (vector<Location>::iterator it = vecpoi.begin(); it != vecpoi.end(); ++it){
		disttemp=havdist(p.lat,p.lon,it->lat,it->lon);
	    disttemp=disttemp*1000;
		if(disttemp<distmin and disttemp<(WDdistance/2)){
			//cout<<(int)it->alt <<" cord "<< p.lat<<" "<<p.lon<<" "<<it->lat <<" " <<it->lon<<endl;
			distmin=disttemp;
			idpoi=(int)it->alt;
		}

	}
	//cout<< idpoi<< " distance "<<distmin<< endl;
	return idpoi;
}




vector<float> simulationOperationsPOI(int num_users,Smartphones smM, Smartphones::iterator it_smM,map <time_t,vector<Event> > contacts, Users usersM, Users::iterator it_usersM,int days,Events eventsL2,Events::iterator it_eventsL2,map <int , pair<Location,Location> > grid,map <int ,vector<Location> > poicords,int numcoluns){


	// WiFi
	double rho_tx=0.27; // WiFi power in transmission (W)
	double rho_id=3.68; // WiFi power in idle mode (W)
	double lambda_g=1000.0; // Rate of generation of packets
	double gamma_g=0.11*pow(10,-3); // Energy cost to elaborate a generated packet (J)
	int wifi_uplink_data_tx=1000000;// WiFi uplink data rate: 1 Mbps
	int datatotali=0;
	double acc_sample_frequency=10; // Hz
	double acc_sample_size=6*8;// bit
	int acc_current=450;// uA

/*	double tem_sample_frequency=50; // Hz
	double tem_sample_size=16;// bit
	int tem_current=1*tem_sample_frequency;*/// uA
	vector<float> results;
	float datatotalitotgiorni=0.0;
	float cons_sensing=0.0;
	float cons_report=0.0;

	pair<float,float> locstop;

	double prox_sample_frequency=10; // Hz
	double prox_sample_size=2*8;// bit
	int prox_current=150;// uA

	int packtotali=0;
	double gps_sample_frequency=0.1; // Hz
	double gps_sample_size=24*8;// bit
	int gps_current=22000;// uA
	float totcontribution=0;
	float quadcontribution=0;
	float batcontrib=0;
	float quadbatcontrib=0;
	float datasent=0;
	float dataexpected;




	// Real consumption per minute, 1- thx while sensing 2- transfer after sensing 3- random thx
	static const float arr[] = {2.22,0.917,1.18,0.561};
	// probabilistic soglia 0.25= 1.22333   0.5= 1.08
	 float pbrep_cost=1.8;
	 float threshold=0.5;
	 //soglia probabilistic
	 float reporting_cost= (2.011*11) /1200;  // costo di report per minute (mah/minute)
	 int bytes_per_minute=40; //kB per minute
	 int kbpermin=109;//kb tx per min
	 float b=1;
	 float upbound=1;
	 float lowbound=0.20;


	vector<float> cons (arr, arr + sizeof(arr) / sizeof(arr[0]) );

    int ctx=0;

    Smartphones active;
    Smartphones stopped;




    float sb;
    float cap;
    DatasentM temp_dtsM;
    DatatbdelM temp_dtbdelM;
    Samples temp_s;
    float old_battery;
    float enTot;

    vector<int> timeroute;

    float ref_lat;
	float ref_lon;
	bool pda=false;

    /* - - - - - - - - - - - - - - - - - - - */
	 // SIMULATION
	 /* - - - - - - - - - - - - - - - - - - - */


	int typeCons=readTypeCons();
    //int typeCons = 0;

	if(typeCons==4){
		typeCons=2;
		pda=true;
	}



	cout << typeCons << endl;
	ofstream outputsimdata;
	ofstream stopfile;
	ofstream activefile;
	ofstream routesfile;
	ofstream  fairfile;
	ofstream  batfairfile;
	ofstream tres;
	ofstream  membersfile;
	ofstream  wdconsfile;
	wdconsfile.open("../data/Outputs/poi/WDconsumption.txt");
	ofstream  memconsfile;
	memconsfile.open("../data/Outputs/poi/ConsumptionMember.txt");
	ofstream  goconsfile;
	goconsfile.open("../data/Outputs/poi/ConsumptionGo.txt");
	ofstream  ngconsfile;
	ngconsfile.open("../data/Outputs/poi/ConsumptionNg.txt");
	membersfile.open("../data/Outputs/poi/Members.txt");
	ofstream  rolefile2;
	rolefile2.open("../data/Outputs/r/roles2.txt",ios::app);
	ofstream  rolefile;
	rolefile.open("../data/Outputs/stat/roles.txt",ios::app);
	ifstream perc;
	ifstream dur;
	ostringstream fileNameStream;                    // let this be empty
	fileNameStream << "../data/Outputs/DataFairness_" << typeCons << ".txt"; // and pass "dice_" here
	string fileName = fileNameStream.str();
	fairfile.open(fileName.c_str(),ios::app);
	ofstream exceedfile;
	ofstream noexceedfile;

	exceedfile.open("../data/Outputs/stat/exceed.txt",ios::app);

	noexceedfile.open("../data/Outputs/stat/noexceed.txt",ios::app);
	ostringstream fileNameStream2;                    // let this be empty
	fileNameStream2 << "../data/Outputs/BatFairness_" << typeCons << ".txt"; // and pass "dice_" here
	string fileName2 = fileNameStream2.str();
	batfairfile.open(fileName2.c_str(),ios::app);
	 outputsimdata.open("../data/Outputs/SimulationData.txt");
	 stopfile.open("../data/Outputs/poi/Groups.txt");
	 activefile.open("../data/Outputs/Active.txt");
	 routesfile.open("../results/Routes.txt");
	 tres.open("../data/Outputs/tres.txt");
	 perc.open("../data/Inputs/perc_connections");
	 dur.open("../data/Inputs/durations");

	 outputsimdata << "/Day/-" << "/Hour/-" << "/Minute/-" << "/ID-User/-" << "/Lat/-"
			 	   << "/Long/- "<< "/BatteryLevel/-" << "/SensorDataInformation/-" << endl;
	 activefile << "/ID-User/-" <<  "/DataGenerated/- "<< "/EnergyConsumption/-"<< "/BatteryConsumption/-"<< "/BatteryLevel/- Minutes of Contribution -/ Timestamp -/ last position" << endl;

	 int currentDay=0;
	 vector<float> percentages;
	 vector<int> duration_hour;

	 bool traces =false;
	 float volt=3.7;
	 int packold=0;


	vector<pair<float, float> > route;
	vector<bool> generate;
	bool gen;
	bool flagstart;
	int numgeneration;
	int duration;
	int ref_user=-1;

	int counteruser=0;
	Smartphone sm(0,0,0,0,"",temp_dtsM,temp_dtbdelM,temp_s,0,route,generate,true,true,0,0,0,0,timeroute,0,0);


	 cout << "**Start simulation**" << endl;
	 if(typeCons==3){

		 float tmp;

		 while(perc >> tmp){
			 percentages.push_back(tmp);
			 //cout<< tmp<<endl;
		 }

		 while(dur >> tmp){
				duration_hour.push_back(tmp);
				//cout<< tmp<<endl;
			 }




	 }

 if(traces==false)
	 kbpermin=((60*wifi_uplink_data_tx)/8); //Bytes

 dataexpected=555.0125*num_users;





int numchanges=0;
int numsamego=0;

float ex_go=0,ex_mem=0,ex_ng=0,ex_nummem=0;
int ex_tot=0;

float no_go=0,no_mem=0,no_ng=0,no_nummem=0;
int no_tot=0;


vector<Event>::iterator it3;

map<int,set<int> > members;
map<int,int> PastMembers;
map<int,int> groupduration;
map<int,int> groupmems;
map <int,vector<Event>::iterator > eves;
map <int,Location> listPoi;
map <int,int > gomap;
map <int,int >::iterator g;
bool newelection=false;
map<int,float> dataPerGroup;
map<int,float> consumptionGo;
map<int,float> dataGoMin;
map<int,int> mapmember;
map<int,int> waitpoi;

for(map <int ,vector<Location> >::iterator pit = poicords.begin(); pit != poicords.end(); ++pit){
	vector<Location>  vtemp=pit->second;
	for (vector<Location>::iterator vit = vtemp.begin(); vit != vtemp.end(); ++vit)
		listPoi[int(vit->alt)]=(*vit);

}


int numtot=0,numalone=0,nummember=0,numgos=0;

for (map <time_t,vector<Event> >::iterator it = contacts.begin(); it != contacts.end(); ++it) {

	vector<Event> evl=it->second;


	members.clear();
	eves.clear();
	mapmember.clear();
	for (vector<Event>::iterator it2 = evl.begin(); it2 != evl.end(); ++it2){

		int oldpoi=PastMembers[it2->id_user];
		int idpoi=ClosestPoi(it2->loc,grid,poicords[it2->quad],oldpoi);


		if (idpoi==-1){
			PastMembers[it2->id_user]=-1;
			it2->status="alone";
			it2->idgo=-2;
		}
		else{
			if(idpoi!= oldpoi){
				int waitingprevision= rand() % 15 +1 ;
				waitpoi[it2->id_user]=waitingprevision;

			}
			waitpoi[it2->id_user]--;
			members[idpoi].insert(it2->id_user);
			PastMembers[it2->id_user]=idpoi;
			eves.insert(pair <int, vector<Event>::iterator >(it2->id_user,it2));
		}


	}


	for(map <int,set<int> >::iterator mit = members.begin(); mit != members.end(); ++mit){


		newelection=false;
		g=gomap.find(mit->first);

		if(g !=gomap.end()){


			set<int>::iterator check= mit->second.find(g->second);
			if( check !=mit->second.end()){
				if(mit->second.size()>1){
				map <int,vector<Event>::iterator >::iterator goev=eves.find((*check));
				gomap[mit->first]=goev->second->id_user;
				goev->second->status="go";
				goev->second->idgo=-1;
				goev->second->members=mit->second.size();

				mapmember[mit->first]=mit->second.size();
				groupduration[mit->first]++;
				numsamego++;



				for(set<int>::iterator sit = mit->second.begin(); sit != mit->second.end(); ++sit){

					//it_smM=smM.find((*sit));
					if((*sit)==goev->second->id_user)
						continue;
					float dist;

					map <int,vector<Event>::iterator >::iterator tev=eves.find((*sit));

					dist=havdist(goev->second->loc.lat,goev->second->loc.lon,tev->second->loc.lat,tev->second->loc.lon);
					dist=dist*1000;
					//cout<<dist<<endl;
					tev->second->distgo=dist;
					tev->second->status="member";
					tev->second->idgo=mit->first;
					//smM[(*sit)].goai=15;
					//cout<<mit->first<<"  "<<((*tev).second.quad) <<"   "<<grid[mit->first].first.lat<<"   "<<grid[mit->first].second.lat<<" "<< (*tev).second.loc.lat<<endl;

				}



				continue;
				}
				else{
					gomap.erase(mit->first);
					newelection=true;

				}
			}
			else{
				gomap.erase(mit->first);
				newelection=true;
			}



		}
		else{
			newelection=true;
		}



		if(newelection==true){
			if(groupduration[mit->first]!=0)
				stopfile<<"Durata "<<groupduration[mit->first]<<" Data "<<dataPerGroup[mit->first]<<" membsAVG "<< groupmems[mit->first]<<endl;

			groupduration[mit->first]=1;
			groupmems[mit->first]=0;
			dataPerGroup[mit->first]=0;
			numchanges++;
			int goaimax=0;
			int idmaxg=-1;
			for(set<int>::iterator sit = mit->second.begin(); sit != mit->second.end(); ++sit){
				//it_smM=smM.find((*sit));
				map <int,vector<Event>::iterator >::iterator tev=eves.find((*sit));
				tev->second->status="member";
				tev->second->idgo=mit->first;
				//smM[(*sit)].goai=15;
				//cout<<mit->first<<"  "<<((*tev).second.quad) <<"   "<<grid[mit->first].first.lat<<"   "<<grid[mit->first].second.lat<<" "<< (*tev).second.loc.lat<<endl;
				//cout<< mit->first<< " id poi" <<listPoi[mit->first].lon <<endl;
				int goai_temp= updateGoaiPoi(smM[(*sit)].battery,waitpoi[tev->second->id_user],smM[(*sit)].smp,tev->second->rssi,tev->second->speed,listPoi[mit->first]);
				smM[(*sit)].goai=goai_temp;
				if(goai_temp>goaimax){
					goaimax=goai_temp;
					idmaxg=(*sit);
				}

			}
			if(idmaxg!=-1){
				map <int,vector<Event>::iterator >::iterator tev2=eves.find(idmaxg);
				gomap[mit->first]=tev2->second->id_user;
				tev2->second->idgo=-1;
				tev2->second->members=mit->second.size();
				tev2->second->status="go";

				mapmember[mit->first]=mit->second.size();


				for(set<int>::iterator sit2 = mit->second.begin(); sit2 != mit->second.end(); ++sit2){
					map <int,vector<Event>::iterator >::iterator evit2=eves.find((*sit2));
					if(evit2->second->idgo>0){
						float distg;

						distg=havdist(tev2->second->loc.lat,tev2->second->loc.lon,evit2->second->loc.lat,evit2->second->loc.lon);
						distg=distg*1000;
						//cout<<distg<<" "<<tev2->second->id_user<<" "<<evit2->second->id_user<<" "<<evit2->second->timestamp.tm_min<<endl;
						evit2->second->distgo=distg;

					}


				}

			}

		}








	}


	sort (evl.begin(), evl.end(), idgocomp);
	consumptionGo.clear();
	dataGoMin.clear();

	for (vector<Event>::iterator it_eventsL = evl.begin(); it_eventsL != evl.end(); ++it_eventsL){




			if(it_eventsL->idgo>0 and dataGoMin[it_eventsL->idgo]>MaxDataGroup){

				//cout<<"Taglio "<<it_eventsL->idgo<<endl;

				mapmember[it_eventsL->idgo]--;
				it_eventsL->idgo=-2;

			}

			numtot++;

			if(it_eventsL->idgo==-2)
				numalone++;

			if(it_eventsL->idgo==-1){




				int idp=PastMembers[it_eventsL->id_user];


				if(mapmember[idp]!=it_eventsL->members){
					it_eventsL->members=mapmember[idp];
					//cout<<"memeber tagliato "<<it_eventsL->id_user<<endl;

				}

				membersfile<<it_eventsL->members<<endl;
				groupmems[idp]=groupmems[idp] + (mapmember[idp] -1);
				//cout<<"GO "<<it_eventsL->id_user<<" data "<<dataPerGroup[idp]<< " members "<< it_eventsL->members<<" min "<<it_eventsL->timestamp.tm_min<<" cons "<<consumptionGo[idp]<<" "<<dataGoMin[idp]<<endl;
				numgos++;






			}







		 if(ref_user!=(*it_eventsL).id_user){ //da mantenere
			 if(ref_user==-1){
				 ref_user=1;
			 }

			else{





				smM.erase(ref_user);
				Smartphone temp_str(ref_user,sm.startBattery,old_battery,cap,"Outdoor",temp_dtbdelM,temp_dtsM,temp_s,sm.stop,route,generate,gen,flagstart,numgeneration,duration,sm.minwalk,sm.goai,timeroute,sm.idgo,sm.members);
				smM.insert(pair <int, Smartphone >(ref_user,temp_str) );

				temp_s.clear();
				temp_dtsM.clear();
				temp_dtbdelM.clear();
				locstop=make_pair(0,0);

			}
		 ref_user=(*it_eventsL).id_user;

		 it_smM=smM.find(ref_user);
		 it_usersM=usersM.find(ref_user);
		 sm=(*it_smM).second;
		 temp_dtbdelM=sm.dtbd;
		 temp_dtsM=sm.dts;

		 temp_s=sm.smp;
		 old_battery=sm.battery;
		 cap=sm.capacity;
		 generate=sm.generate;
		 gen=sm.gen;
		 duration=sm.duration;
		 numgeneration=sm.numgeneration;
		 flagstart=sm.flagstart;
		 route=sm.route;
		 timeroute=sm.timeroute;

		 int ref_smartphone=sm.id_user;
		 string ref_context=sm.context;

		 }


		 ref_lat=(*it_eventsL).loc.lat;
		 ref_lon=(*it_eventsL).loc.lon;
		 float ref_alt=(*it_eventsL).loc.alt;









		 tm prev_timestamp;
		 Samples::iterator it_tmpsample;

		 it_tmpsample= --temp_s.end();





		 if(currentDay!=(*it_eventsL).timestamp.tm_wday){
		 				 int userusati=ref_user-counteruser;
		 				 results.push_back(datatotali/userusati);
		 				 results.push_back(cons_sensing/userusati);
		 				 results.push_back(cons_report/userusati);
		 				 cons_report=0;
		 				 cons_sensing=0;
		 				 datatotali=0;

		 				 counteruser=ref_user;
		 			 }
		 //inserted to avoid for
		 currentDay=(*it_eventsL).timestamp.tm_wday;

		 prev_timestamp=(*it_tmpsample).pos.timestamp;
		 //serve???
		 //prev_timestamp.tm_wday = currentDay;

		 // if sample list empty previous timestamp = actual event ts
		 if(temp_s.size() == 0){
			 prev_timestamp=(*it_eventsL).timestamp;
		 }

		 // edited first event of the day has to be 0 as difference of minutes between samples
		 int time_between_samples = 0;
		 if((*it_eventsL).timestamp.tm_wday == prev_timestamp.tm_wday){
			 if((*it_eventsL).timestamp.tm_hour == prev_timestamp.tm_hour){
				 time_between_samples = abs((*it_eventsL).timestamp.tm_min-prev_timestamp.tm_min);
			 }


			 if((*it_eventsL).timestamp.tm_hour == prev_timestamp.tm_hour+1){
				 time_between_samples = ((*it_eventsL).timestamp.tm_min+(60 - prev_timestamp.tm_min));
			 }


			 //non dovrebbe entrarci mai?
			 if((*it_eventsL).timestamp.tm_hour > prev_timestamp.tm_hour+1){
				 int hours_counter2 = (*it_eventsL).timestamp.tm_hour - prev_timestamp.tm_hour ;
				 time_between_samples = hours_counter2*60+abs(((*it_eventsL).timestamp.tm_min-prev_timestamp.tm_min));
			 }
		 }

		 /*** GENERATION OF NEW SAMPLES ***/
		 if(flagstart==true){
			 time_between_samples=0;
		 }
		 else{
			 time_between_samples=1;
		 }


		 sm.minwalk=sm.minwalk + time_between_samples;


		 // Always on sensors: consider sampling frequency, take time between previous timestamp and current
		 // and generate a number of samples accordingly

		 float temp_value=fRand(1.0,5.0);
		 float temp_size=fRand(500.0, 2000.0); // Kilobyte
		 string temp_type="GPS";

		 // GPS
		 Location temp_loc(ref_lat,ref_lon,ref_alt);
		 Position temp_p(temp_loc,(*it_eventsL).timestamp);
		 Sample temp_sample(temp_type,temp_value,temp_size,false,temp_p);
		 temp_s.push_back(temp_sample);


			 float tau_tx=lambda_g*(0.000192+((28.0+1500.0)/wifi_uplink_data_tx));// 192 us: PLCP time + (Header+Payload)/data rate
			 float wifi_power=rho_id+rho_tx*tau_tx+lambda_g*gamma_g;// power to tx one packet

		// * * ACCELLEROMETER
			 int acc_num_samples_to_generate=acc_sample_frequency*60*time_between_samples;
			 int acc_data_size=(acc_num_samples_to_generate*acc_sample_size)/8;// to have Bytes
			 int acc_energy_sampling=acc_current*(60*time_between_samples);// these are uAs (remember: 3600 uAs= 1uAh)
			 float acc_energy_sampling_hour=acc_energy_sampling/3600000.00;// mAh

		int acc_packets=(int)acc_data_size/1500;
		float acc_tx_time=(acc_data_size*8.0)/wifi_uplink_data_tx;// in seconds (bit / bps)

		float acc_wifi_energy=wifi_power*acc_packets; // Joules (Watt*seconds)
		float acc_wifi_consumption=( acc_wifi_energy/( 1000*3.6 ) )*( 1000/volt ); // mah

		outputsimdata << currentDay << " "<<(*it_eventsL).timestamp.tm_hour << " "
					  << (*it_eventsL).timestamp.tm_min << " "
					  << sm.id_user << " "
					  << ref_lat << " "// position latitude
					  << ref_lon << " "// position longitude
					  << sm.battery << " "
					  << "Accelerometer" << " "
					  << acc_num_samples_to_generate << " "
					  << acc_data_size << " "// in Bytes
					  << acc_packets << " "// number of packets
					  << acc_tx_time << " "// uplink transmission time in seconds
					  << acc_energy_sampling_hour << " "// in uAh
					  << acc_wifi_energy//
					  << endl;

		// * * TEMPERATURE
		int gps_num_samples_to_generate=gps_sample_frequency*60*time_between_samples;
		int gps_data_size=(gps_num_samples_to_generate*gps_sample_size)/8;// to have Bytes
		int gps_energy_sampling=gps_current*(60*time_between_samples)*gps_sample_frequency;// these are uAs (remember: 3600 uAs= 1uAh)
		float gps_energy_sampling_hour=gps_energy_sampling/3600000.00;// uAh

		int gps_packets=(int)gps_data_size/1500;
		float gps_tx_time=(gps_data_size*8.0)/wifi_uplink_data_tx;// in seconds (Byte / Bps)
		float gps_wifi_energy=wifi_power*gps_packets; // Joules (Watt*seconds)

		float gps_wifi_consumption=( gps_wifi_energy/( 1000*3.6 ) )*( 1000/volt );

		outputsimdata << currentDay << " " << (*it_eventsL).timestamp.tm_hour << " "
					  << (*it_eventsL).timestamp.tm_min << " "
					  << sm.id_user << " "
					  << ref_lat << " "// position latitude
					  << ref_lon << " "// position longitude
					  << sm.battery << " "
					  << "GPS" << " "
					  << gps_num_samples_to_generate << " "
					  << gps_data_size << " "// in Bytes
					  << gps_packets << " "
					  << gps_tx_time << " "// uplink transmission time in seconds
					  << gps_energy_sampling_hour << " "// in mAh
					  << gps_wifi_energy
					  << endl;

		// * * PRESSURE
		int prox_num_samples_to_generate=prox_sample_frequency*60*time_between_samples;
		int prox_data_size=(prox_num_samples_to_generate*prox_sample_size)/8;// to have Bytes
		int prox_energy_sampling=prox_current*(60*time_between_samples);// these are uAs (remember: 3600 uAs= 1uAh)
		float prox_energy_sampling_hour=prox_energy_sampling/3600000.00;// mAh

		int prox_packets=(int)prox_data_size/1500;
		float prox_tx_time=(prox_data_size*8.0)/wifi_uplink_data_tx;// in seconds (Byte / Bps)
		float prox_wifi_energy=wifi_power*prox_packets; // Joules (Watt*seconds)
		float prox_wifi_consumption=( prox_wifi_energy/( 1000*3.6 ) )*( 1000/volt );

		outputsimdata << currentDay << " " << (*it_eventsL).timestamp.tm_hour << " "
					  << (*it_eventsL).timestamp.tm_min << " "
					  << sm.id_user << " "
					  << ref_lat << " "// position latitude
					  << ref_lon << " "// position longitude
					  << sm.battery << " "
					  << "Proximity" << " "
					  << prox_num_samples_to_generate << " "
					  << prox_data_size << " "// in Bytes
					  << prox_packets << " "
					  << prox_tx_time << " "//uplink transmission time in seconds
					  << prox_energy_sampling_hour << " "// in mAh
					  << prox_wifi_energy
					  << endl;


		 temp_dtbdelM["Accelerometer"]+=acc_data_size; // number of Bytes
		 temp_dtbdelM["GPS"]+=1.0;
		 float tot_data=0;

		 float cons_sens=0;
		 float cons_wifi=0;

		 if(sm.stop==0){
			 tot_data=acc_data_size+prox_data_size+gps_data_size;
			 temp_dtbdelM["data"]+=tot_data;
			 cons_sens=prox_energy_sampling_hour+gps_energy_sampling_hour+acc_energy_sampling_hour;
			 temp_dtbdelM["consumption-data"]+=cons_sens;

			 int packs=(int)tot_data/1500;
			 packs+=1;
			 if(typeCons==0)
				 packtotali +=packs;
			 float tot_wifi_energy=wifi_power*packs; // Joules (Watt*seconds)
			 cons_wifi=( tot_wifi_energy/( 1000*3.6 ) )*( 1000/volt );
			 temp_dtbdelM["consumption-wifi"]+=cons_wifi;
			 temp_dtbdelM["Energy"]+=cons_sens;
		}
	/*			 cout<< acc_data_size<<" " << prox_data_size<<" "<< gps_data_size<<endl;

		 cout<<"DAI    " <<cons_wifi<<endl;*/



			 enTot= cons[typeCons]*temp_dtbdelM["Minutes"];

			 if(traces==false){
				 enTot=temp_dtbdelM["consumption-data"];
				 if(typeCons==0){
					 enTot+=temp_dtbdelM["consumption-wifi"];
					 temp_dtbdelM["Energy"]+=cons_wifi;
				 }
					 }

			 float enmax= (cap/100)*bat_threshold;




			 if(pda==true){
				 float si=datasent/dataexpected;
				 if(b>0 ){
					 if(si>upbound){
						 if(b>1)
							 b=b-1;

						 else
							 b=b/(1+b);

					 }
				 }
				if(b<50 ){
					 if(si<lowbound){
						 if(b<1)
							 b=b/(1-b);
						 else
							 b=b+1;
					 }
				 }
				 b=0.5;
				 threshold=1-pow(si,b);
				 tres<< threshold<<" "<<datasent<<" "<<b<<" "<<si<<" "<<pow(si,b) <<endl;
			 }





			 if(typeCons==2){
				 float prob=(float(rand()) /float(RAND_MAX));
				 if(threshold<prob)
					 gen=false;
				 else
					 gen=true;

			 }

			 if(typeCons==3 && duration==0){
				 float prob=(float(rand()) /float(RAND_MAX));
				 if(percentages[((*it_eventsL).timestamp.tm_hour)]<prob)
					 gen=false;
				 else{
					 duration=duration_hour[((*it_eventsL).timestamp.tm_hour)];
					 gen=true;
				 }






			 }

			 if(duration>0)
				 duration-=time_between_samples;

			 if(typeCons==1)
				 gen=true;

			 if(typeCons==0){

				 if(enTot>=enmax && sm.stop==0){

					 sm.stop=1;


					 sb= sm.startBattery;
	/*					 if(typeCons==1){
						 ctx=temp_dtbdelM["Minutes"] * reporting_cost;

						 enTot+=ctx;



					 }*/

					old_battery=sm.battery;
					float rem= (cap/100)*old_battery;
					rem = rem - enTot;

					old_battery= (rem/cap)*100;
					locstop=make_pair(ref_lon,ref_lat);

					//stopfile << ref_user <<" " << temp_dtbdelM["Total"] <<" "<< enTot <<" "<< sb-old_battery <<" "<<old_battery << " "<<(*(it_eventsL)).loc.lon<<" "<<(*(it_eventsL)).loc.lat<<endl;
					 //cout<< "Stopped"<<" "<<ref_user<<endl;
					gen=false;
				 }
				 if(sm.stop==0)
					 temp_dtbdelM["Minutes"]+=time_between_samples;
			 }


				float temp_data=0.0;

				 if(gen || typeCons!=0){
					 if(traces==true)
					 if(typeCons!=0)
					 temp_dtbdelM["Minutes"]+=time_between_samples;

					 temp_dtbdelM["Total"]+=(time_between_samples * bytes_per_minute);
					 temp_data=tot_data;
					 if((typeCons==2 || typeCons==3) && gen ){
						 if(traces==true){
							 if(temp_dtbdelM["Total"]<kbpermin){
								 temp_dtsM["Total"]+=temp_dtbdelM["Total"];
								 datasent+=temp_dtbdelM["Total"];
								 temp_dtbdelM["Total"]=0;

							 }
							 else{
								 temp_dtsM["Total"]+=kbpermin;
								 temp_dtbdelM["Total"]-=kbpermin;
								 datasent+=kbpermin;
							 }
						 }
						 else{
							 temp_dtbdelM["Minutes"]+=time_between_samples;

							 if(temp_dtbdelM["data"]<kbpermin){

								 temp_dtsM["Total"]+=temp_dtbdelM["data"];
								 temp_data=temp_dtbdelM["data"];
								 int tot_packets=(int)temp_dtbdelM["data"]/1500; //Bytes/dimension of one packet
								 tot_packets+=1;
								 float wifitemp=((tot_packets*wifi_power)/( 1000*3.6 ) )*( 1000/volt);
								 temp_dtbdelM["Energy"]+=wifitemp;
								 packtotali+=tot_packets;
								 temp_dtbdelM["data"]=0;


							 }

							 else{

								 temp_dtsM["Total"]+=kbpermin;
								 temp_dtbdelM["data"]-=kbpermin;
								 temp_data=kbpermin;
								 int tot_packets=(int)kbpermin/1500;//Bytes/dimension of one packet
								 tot_packets+=1;
								 packtotali+=tot_packets;
								 temp_dtbdelM["Energy"]+=((tot_packets*wifi_power)/( 1000*3.6 ) )*( 1000/volt);

							 }



						 }



					 }
				 }
	//			 if(ref_user==42)
	//				 cout<< ref_lon<<" "<<ref_lat<<endl;
		 route.push_back(make_pair(ref_lon,ref_lat));
		 timeroute.push_back((*it_eventsL).timestamp.tm_min);

		 generate.push_back(gen);
		 if(gen==true && flagstart==false)
			 numgeneration+=1;

		 float datiTemp;
		 if(traces==false){

			datiTemp= temp_data/1000;    // from Byte to KB
			//enTot=temp_dtbdelM["consumption-wifi"]+ temp_dtbdelM["consumption-data"];
			//cons_sensing+=temp_dtbdelM["consumption-data"];
			//cons_report+=temp_dtbdelM["consumption-wifi"];
			if(datiTemp==0){
				exit(-1);
			}

			if(it_eventsL->idgo!=-2)
				dataPerGroup[PastMembers[ref_user]]+=datiTemp;

		}






		 if(flagstart)
			 flagstart=false;


		 float upwd_perbit,uplte_perbit,downwd_perbit,thrwd,thrlte;

		 thrlte=it_eventsL->rssi;
		 //cout<< uplte_perbit <<" "<< upwd_perbit<<" " <<tot_data*8*downwd_perbit<<" "<< tot_wifi_energy <<endl;


		 if(it_eventsL->idgo>0){


			 float dt=it_eventsL->distgo;

			 for(int ij=0;ij<5;ij++){
				 if(dt<distthr[ij]){
					 thrwd=thr[ij];
					 break;
				 }
			 }

			upwd_perbit = (283.17* thrwd + 132.86)/(thrwd*wifi_uplink_data_tx*1000);


			//uplte_perbit = (438.39* thrwd + 1288.04)/(thrlte*wifi_uplink_data_tx*1000);

			downwd_perbit= (137.01* thrwd + 132.86)/(thrwd*wifi_uplink_data_tx*1000);


			consumptionGo[it_eventsL->idgo]= consumptionGo[it_eventsL->idgo] + (1000*8*datiTemp*downwd_perbit);
			dataGoMin[it_eventsL->idgo]= dataGoMin[it_eventsL->idgo] + datiTemp;

			temp_dtbdelM["wdcons"]+= 1000*8*datiTemp*upwd_perbit;


			//cout<<"member "<<it_eventsL->id_user <<"  GO "<<gomap[it_eventsL->idgo]<<" quad "<<PastMembers[it_eventsL->id_user]<<" data "<< dataPerGroup[it_eventsL->idgo]<<" min "<<it_eventsL->timestamp.tm_min<<" datitemp "<< datiTemp <<" dist "<< it_eventsL->distgo <<" membs "<<mapmember[it_eventsL->idgo]<<endl;
			nummember++;

			memconsfile<<1000*8*datiTemp*upwd_perbit<<endl;
			temp_dtbdelM["member"]+=1;

		 }



		if(it_eventsL->idgo==-1){
			    uplte_perbit = (438.39* thrlte + 1288.04)/(thrlte*wifi_uplink_data_tx*1000);

			    temp_dtbdelM["wdcons"]+=consumptionGo[PastMembers[it_eventsL->id_user]];
			    temp_dtbdelM["wdcons"]+=uplte_perbit*1000*8*(datiTemp + dataGoMin[PastMembers[it_eventsL->id_user]]);
			    goconsfile<<consumptionGo[PastMembers[it_eventsL->id_user]]+(uplte_perbit*1000*8*(datiTemp + dataGoMin[PastMembers[it_eventsL->id_user]]))<<endl;
			    temp_dtbdelM["go"]+=1;
			    temp_dtbdelM["nummember"]+=it_eventsL->members;
		}

		if(it_eventsL->idgo==-2){
				uplte_perbit = (438.39* thrlte + 1288.04)/(thrlte*wifi_uplink_data_tx*1000);
				temp_dtbdelM["wdcons"]+=NoNoGroup*uplte_perbit*(1000*8*datiTemp);
				ngconsfile<<uplte_perbit*(1000*8*datiTemp)<<endl;
				temp_dtbdelM["alone"]+=1;

		}




		 if(sm.minwalk==(*it_usersM).second.minutes){


			 enTot=0;

			 float dati;
			 sb=sm.startBattery;

			 if(typeCons==0 || typeCons==1)
				 dati=temp_dtbdelM["Total"];

			 if(typeCons==2 || typeCons==3)
				 dati=temp_dtsM["Total"];


			 if(typeCons==1 ){

				ctx=dati * reporting_cost;
				enTot+=ctx;
			}

			if(typeCons==3 || pda==true ){

				ctx=numgeneration * pbrep_cost;
				enTot+=ctx;
				enTot+= cons[3]*(temp_dtbdelM["Minutes"]-numgeneration);
				cons_sensing+=cons[3]*(temp_dtbdelM["Minutes"]-numgeneration);
				cons_report+=ctx;
			}
			else{
				enTot+= cons[typeCons]*temp_dtbdelM["Minutes"];
				cons_sensing+=cons[3]*temp_dtbdelM["Minutes"];
				cons_report+=enTot-cons[3]*temp_dtbdelM["Minutes"];
									}

			if(traces==false){

				dati= temp_dtbdelM["data"]/1000;    // from Byte to KB
				enTot=temp_dtbdelM["consumption-wifi"]+ temp_dtbdelM["consumption-data"];
				cons_sensing+=temp_dtbdelM["consumption-data"];
				cons_report+=temp_dtbdelM["consumption-wifi"];

				if(typeCons==2 || typeCons==3){
					dati=dati=temp_dtsM["Total"]/1000;
					enTot=temp_dtbdelM["Energy"];
					cons_sensing+=temp_dtbdelM["consumption-data"];
					cons_report+=enTot-temp_dtbdelM["consumption-data"];

				}
			}









			float totalwdconsumption=0.0;


			totalwdconsumption=temp_dtbdelM["wdcons"];




			wdconsfile<<it_eventsL->id_user<<" "<<it_eventsL->idgo<<" "<<totalwdconsumption<<endl;



			if(totalwdconsumption>2.7){
				ex_go+=(temp_dtbdelM["go"]/sm.minwalk)*100;
				ex_ng+=(temp_dtbdelM["alone"]/sm.minwalk)*100;
				ex_mem+=(temp_dtbdelM["member"]/sm.minwalk)*100;
				ex_nummem+=temp_dtbdelM["nummember"]/temp_dtbdelM["go"];
				ex_tot+=1;

				}
								//exceedfile<<(temp_dtbdelM["alone"]/sm.minwalk)*100<<" "<<(temp_dtbdelM["go"]/sm.minwalk)*100<<" "<<(temp_dtbdelM["member"]/sm.minwalk)*100<<" "<<temp_dtbdelM["nummember"]/temp_dtbdelM["go"]<<endl;
			else{


				no_go+=(temp_dtbdelM["go"]/sm.minwalk)*100;
				no_ng+=(temp_dtbdelM["alone"]/sm.minwalk)*100;
				no_mem+=(temp_dtbdelM["member"]/sm.minwalk)*100;
				if(temp_dtbdelM["go"]!=0)
					no_nummem+=temp_dtbdelM["nummember"]/temp_dtbdelM["go"];
				no_tot+=1;

			}

			if(it_eventsL->idgo==-1){
				stopfile<<"Durata "<<groupduration[PastMembers[ref_user]]<<" Data "<<dataPerGroup[PastMembers[ref_user]]<<" membsAVG "<<groupmems[PastMembers[ref_user]]<<endl;
				groupduration[it_eventsL->quad]=0;
				groupmems[it_eventsL->quad]=0;
				dataPerGroup[it_eventsL->quad]=0;
			}


			old_battery=sm.battery;
			float rem= (cap/100)*old_battery;
			rem = rem - enTot;

			old_battery= (rem/cap)*100;
			smM.erase(ref_user);
			Smartphone temp_str(ref_user,sb,old_battery,cap,"Outdoor",temp_dtbdelM,temp_dtsM,temp_s,sm.stop,route,generate,gen,flagstart,numgeneration,duration,sm.minwalk,sm.goai,timeroute,sm.idgo,sm.members);
			smM.insert(pair <int, Smartphone >(ref_user,temp_str) );

			if(locstop.first==0){
				locstop=make_pair(ref_lon,ref_lat);
			}
			datatotali+=dati;
			datatotalitotgiorni+=dati;

			float mintotali=(*it_usersM).second.minutes;
			if(mintotali!=0){
			float contrib=numgeneration/mintotali;
			float batteryrel= enTot/(rem + enTot);
			batcontrib+=batteryrel;
			quadbatcontrib+=batteryrel*batteryrel;
			totcontribution +=contrib;
			quadcontribution+=contrib*contrib;
			}
			activefile << sm.id_user <<" "<< dati<<" "<<enTot <<" "<< sb-old_battery <<" "<<old_battery << " "<<numgeneration << " "<< (*(it_eventsL)).timestamp.tm_hour<<","<<(*(it_eventsL)).timestamp.tm_min<< " "<<locstop.first<<" "<<locstop.second <<" "<< temp_dtbdelM["consumption-wifi"]<< " "<<(*it_usersM).second.minutes << endl;

			int k=0;
			//if(ref_user==23 || ref_user==85||ref_user==119 ||ref_user==53 ||ref_user==106 )
			float datigenpermin=dati/numgeneration;
			float dg=0.0;
			if((*(it_eventsL)).timestamp.tm_hour==12 && (*(it_eventsL)).timestamp.tm_wday==0)
			for(int i=0;i<(route.size()-1);i++){
				dg=0.0;
				if(generate[i+1]==true){
					dg=datigenpermin;
					routesfile <<timeroute[i]<<"	"<<route[i].first<<" "<<route[i].second<< "	"<< generate[i+1] <<"	"<<dg<< endl;
				}
			}


		 }


	}



 }

		 rolefile2<<"2 "<<((float)numalone/(float)numtot)*100<<" "<<((float)numgos/(float)numtot)*100<<" "<<((float)nummember/(float)numtot)*100<<endl;
		 rolefile<<"2 "<<((float)numalone/(float)numtot)*100<<" "<<((float)numgos/(float)numtot)*100<<" "<<((float)nummember/(float)numtot)*100<<endl;
		 cout <<"DATA Generated   "<<datatotali<<endl;
		 fairfile <<"tot Contribution  "<<totcontribution<<" quad contrib  "<<quadcontribution<<" Fairness  "<<((totcontribution*totcontribution)/quadcontribution)/num_users<<endl;

		 batfairfile <<"BATTERY CONTRIBUTION  "<<batcontrib<<" quad contrib  "<<quadbatcontrib<<" Fairness  "<<((batcontrib*batcontrib)/quadbatcontrib)/num_users<<endl;

		 cout <<"cambiamenti  "<<numchanges<<" stesso go  "<<numsamego<<" User Alone  "<<numalone<<endl;
		 exceedfile<<"2 "<<ex_ng/ex_tot<<" "<<ex_mem/ex_tot<<" "<<ex_go/ex_tot<<" "<<ex_nummem/ex_tot<<endl;
		 noexceedfile<<"2 "<<no_ng/no_tot<<" "<<no_mem/no_tot<<" "<<no_go/no_tot<<" "<<no_nummem/no_tot<<endl;

		 int userusati2=ref_user-counteruser;
		 results.push_back(datatotali/userusati2);
		 results.push_back(cons_sensing/userusati2);
		 results.push_back(cons_report/userusati2);
		 cons_report=0;
		 cons_sensing=0;
		 datatotali=0;

		counteruser=ref_user;

		perc.close();
	    outputsimdata.close();
	    stopfile.close();
	    activefile.close();

	    return results;




}


int computeCkt(Location loc1,Samples oldsmp1,Location loc2,Samples oldsmp2,float speed1,float speed2){
	float dist;
	dist=(float)havdist(loc1.lat,loc1.lon,loc2.lat,loc2.lon);
	dist =dist*1000;
	if(dist>WDdistance)
		return -1;

	if(oldsmp1.size()!=0 and oldsmp2.size()!=0){

	Samples::iterator sit1=--oldsmp1.end();
	Samples::iterator sit2=--oldsmp2.end();
	double lat2out;
	double lon2out;
	double lat1out;
	double lon1out;
	double bearing1,bearing2;

	float lat1 = sit1->pos.loc.lat * pi / 180;
	float lat2 = loc1.lat * pi / 180;

	float long1 = sit1->pos.loc.lon * pi / 180;
	float long2 = loc1.lon * pi / 180;



	if(speed1==0){
		lon1out=loc1.lon;
		lat1out=loc1.lat;
	}
	else{
		bearing1= atan2(sin(long2-long1) * cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(long2-long1));
		bearing1 = bearing1 * 180 / pi;
		bearing1 = int(bearing1 + 360 ) % 360;

		//cout<<"bearing   " <<bearing1 << " "<<lat1<<" "<<sit1->pos.loc.lon<<" "<<long1<<" "<<sit1->pos.loc.lon<<endl;
		double dist1=speed1*60;

		destVincenty(loc1.lat,loc1.lon, bearing1,dist1,&lat1out, &lon1out);
	}
	//cout<<"vincenty   " <<loc1.lat << " "<<loc1.lon << " "<<bearing1 << " "<<lat1out << " "<<lon1out<<endl;

	if(speed2==0){
		lon2out=loc2.lon;
		lat2out=loc2.lat;
	}
	else{
	lat1 = sit2->pos.loc.lon * pi / 180;
	lat2 = loc2.lat * pi / 180;

	long1 = sit2->pos.loc.lon * pi / 180;
	long2 = loc2.lon * pi / 180;
	double dist2=speed2*60;

	bearing2= atan2(sin(long2-long1) * cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(long2-long1));
	bearing2 = bearing2 * 180 / pi;
	bearing2 = int(bearing2 + 360 ) % 360;

	destVincenty(loc2.lat,loc2.lon, bearing2,dist2,&lat2out, &lon2out);
	}

	dist=(float)havdist(lat2out,lon2out,lat1out,lon1out);
	dist =dist*1000;
	if(dist>WDdistance)
		return -2;
	else
		return 0;
	}
	return -2;
}



int updateGoaiDYN(int C,float battery,int realrssi){


	float wB=constantwB;
	float wR= constantwR;
	float wc=constantwC;
	int goai;
	float rssi;

	rssi=(realrssi/5)*100;

	if(C>10)
		C=10;
	C=C*10;
	goai=wB*battery +wR *rssi + wc* C;

	return goai;
}


bool idgocomp (Event i,Event j) { return (i.idgo>j.idgo); }


vector<float> simulationOperationsDYN(int num_users,Smartphones smM, Smartphones::iterator it_smM,map <time_t,vector<Event> > contacts, Users usersM, Users::iterator it_usersM,int days,Events eventsL2,Events::iterator it_eventsL2,map <int , pair<Location,Location> > grid,int numcolumns){


	// WiFi
	double rho_tx=0.27; // WiFi power in transmission (W)
	double rho_id=3.68; // WiFi power in idle mode (W)
	double lambda_g=1000.0; // Rate of generation of packets
	double gamma_g=0.11*pow(10,-3); // Energy cost to elaborate a generated packet (J)
	int wifi_uplink_data_tx=1000000;// WiFi uplink data rate: 1 Mbps
	int datatotali=0;
	double acc_sample_frequency=10; // Hz
	double acc_sample_size=6*8;// bit
	int acc_current=450;// uA

/*	double tem_sample_frequency=50; // Hz
	double tem_sample_size=16;// bit
	int tem_current=1*tem_sample_frequency;*/// uA
	vector<float> results;
	float datatotalitotgiorni=0.0;
	float cons_sensing=0.0;
	float cons_report=0.0;

	pair<float,float> locstop;

	double prox_sample_frequency=10; // Hz
	double prox_sample_size=2*8;// bit
	int prox_current=150;// uA

	int packtotali=0;
	double gps_sample_frequency=0.1; // Hz
	double gps_sample_size=24*8;// bit
	int gps_current=22000;// uA
	float totcontribution=0;
	float quadcontribution=0;
	float batcontrib=0;
	float quadbatcontrib=0;
	float datasent=0;
	float dataexpected;




	// Real consumption per minute, 1- thx while sensing 2- transfer after sensing 3- random thx
	static const float arr[] = {2.22,0.917,1.18,0.561};
	// probabilistic soglia 0.25= 1.22333   0.5= 1.08
	 float pbrep_cost=1.8;
	 float threshold=0.5;
	 //soglia probabilistic
	 float reporting_cost= (2.011*11) /1200;  // costo di report per minute (mah/minute)
	 int bytes_per_minute=40; //kB per minute
	 int kbpermin=109;//kb tx per min
	 float b=1;
	 float upbound=1;
	 float lowbound=0.20;


	vector<float> cons (arr, arr + sizeof(arr) / sizeof(arr[0]) );

    int ctx=0;

    Smartphones active;
    Smartphones stopped;




    float sb;
    float cap;
    DatasentM temp_dtsM;
    DatatbdelM temp_dtbdelM;
    Samples temp_s;
    float old_battery;
    float enTot;

    vector<int> timeroute;

    float ref_lat;
	float ref_lon;
	bool pda=false;

    /* - - - - - - - - - - - - - - - - - - - */
	 // SIMULATION
	 /* - - - - - - - - - - - - - - - - - - - */


	int typeCons=readTypeCons();
    //int typeCons = 0;

	if(typeCons==4){
		typeCons=2;
		pda=true;
	}



	cout << typeCons << endl;
	ofstream outputsimdata;
	ofstream stopfile;
	ofstream activefile;
	ofstream routesfile;
	ofstream  fairfile;
	ofstream  batfairfile;
	ofstream  membersfile;
	membersfile.open("../data/Outputs/dyn/Members.txt");
	ofstream  wdconsfile;
	wdconsfile.open("../data/Outputs/dyn/WDconsumption.txt");
	ofstream  memconsfile;
	memconsfile.open("../data/Outputs/dyn/ConsumptionMember.txt");
	ofstream  goconsfile;
	goconsfile.open("../data/Outputs/dyn/ConsumptionGo.txt");
	ofstream  ngconsfile;
	ngconsfile.open("../data/Outputs/dyn/ConsumptionNg.txt");

	ofstream  rolefile3;
	rolefile3.open("../data/Outputs/r/roles3.txt",ios::app);
	ofstream  rolefile;
	rolefile.open("../data/Outputs/stat/roles.txt",ios::app);
	ofstream exceedfile;
	ofstream noexceedfile;

	exceedfile.open("../data/Outputs/stat/exceed.txt",ios::app);

	noexceedfile.open("../data/Outputs/stat/noexceed.txt",ios::app);
	ofstream tres;
	ifstream perc;
	ifstream dur;
	ostringstream fileNameStream;                    // let this be empty
	fileNameStream << "../data/Outputs/DataFairness_" << typeCons << ".txt"; // and pass "dice_" here
	string fileName = fileNameStream.str();
	fairfile.open(fileName.c_str(),ios::app);

	ostringstream fileNameStream2;                    // let this be empty
	fileNameStream2 << "../data/Outputs/BatFairness_" << typeCons << ".txt"; // and pass "dice_" here
	string fileName2 = fileNameStream2.str();
	batfairfile.open(fileName2.c_str(),ios::app);
	 outputsimdata.open("../data/Outputs/SimulationData.txt");
	 stopfile.open("../data/Outputs/dyn/Groups.txt");
	 activefile.open("../data/Outputs/Active.txt");
	 routesfile.open("../results/Routes.txt");

	 perc.open("../data/Inputs/perc_connections");
	 dur.open("../data/Inputs/durations");

	 outputsimdata << "/Day/-" << "/Hour/-" << "/Minute/-" << "/ID-User/-" << "/Lat/-"
			 	   << "/Long/- "<< "/BatteryLevel/-" << "/SensorDataInformation/-" << endl;
	 activefile << "/ID-User/-" <<  "/DataGenerated/- "<< "/EnergyConsumption/-"<< "/BatteryConsumption/-"<< "/BatteryLevel/- Minutes of Contribution -/ Timestamp -/ last position" << endl;

	 int currentDay=0;
	 vector<float> percentages;
	 vector<int> duration_hour;

	 bool traces =false;
	 float volt=3.7;
	 int packold=0;


	vector<pair<float, float> > route;
	vector<bool> generate;
	bool gen;
	bool flagstart;
	int numgeneration;
	int duration;
	int ref_user=-1;

	int counteruser=0;
	Smartphone sm(0,0,0,0,"",temp_dtsM,temp_dtbdelM,temp_s,0,route,generate,true,true,0,0,0,0,timeroute,0,0);
	Smartphone sm2(0,0,0,0,"",temp_dtsM,temp_dtbdelM,temp_s,0,route,generate,true,true,0,0,0,0,timeroute,0,0);
	Smartphone smtemp(0,0,0,0,"",temp_dtsM,temp_dtbdelM,temp_s,0,route,generate,true,true,0,0,0,0,timeroute,0,0);
	Smartphones::iterator it_smMtemp;

	 cout << "**Start simulation**" << endl;
	 if(typeCons==3){

		 float tmp;

		 while(perc >> tmp){
			 percentages.push_back(tmp);
			 //cout<< tmp<<endl;
		 }

		 while(dur >> tmp){
				duration_hour.push_back(tmp);
				//cout<< tmp<<endl;
			 }




	 }

 if(traces==false)
	 kbpermin=((60*wifi_uplink_data_tx)/8); //Bytes

 dataexpected=555.0125*num_users;





int numchanges=0;
int numsamego=0;

float ex_go=0,ex_mem=0,ex_ng=0,ex_nummem=0;
int ex_tot=0;

float no_go=0,no_mem=0,no_ng=0,no_nummem=0;
int no_tot=0;

vector<Event>::iterator it3;

map<int,set<int> > members;

map <int,vector<Event>::iterator > eves;
map <int,vector<Event>::iterator > eves2;
map <int,vector<Event>::iterator >::iterator g;
map<int,float> dataPerGroup;
map<int,float> consumptionGo;
map<int,float> dataGoMin;
map<int,int> durationGroup;
map<int,int> groupmems;
map<int,int> mapmember;

//set<int> gofinished;
double dist;



bool newelection=false;

int numtot=0,numalone=0,nummember=0,numgos=0;

float consmembs=0.0;
for (map <time_t,vector<Event> >::iterator it = contacts.begin(); it != contacts.end(); ++it) {

	vector<Event> evl=it->second;



	members.clear();
	eves.clear();
	eves2.clear();
	mapmember.clear();

	for (vector<Event>::iterator it2 = evl.begin(); it2 != evl.end(); ++it2){

		members[it2->quad].insert(it2->id_user);
		eves.insert(pair <int,vector<Event>::iterator >(it2->id_user ,it2));
		//cout<<it2->quad <<"   "<<grid[it2->quad].first.lat<<"   "<<grid[it2->quad].second.lat<<" "<< (*it2).loc.lat<<endl;
		//if(it2->id_user==1257 or it2->id_user==1066 or it2->id_user==1274)
			//cout<<"A0 "<<it2->id_user<<" "<<endl;

	}


	for(map <int,vector<Event>::iterator >::iterator evit = eves.begin(); evit != eves.end(); ++evit){

		vector<Event>::iterator currevent=evit->second;


		Smartphone smem,sgo;
		smem=smM[evit->first];
		int idgo=smem.idgo;
		currevent->idgo=idgo;

		//if(evit->first==1257 or evit->first==1066 or evit->first==1274)
			//cout<<"A "<<evit->first<<" "<<endl;


			if(idgo != -1){

				if(idgo>0){
					g=eves.find(idgo);

					if(g!=eves.end()){
						sgo=smM[idgo];
						dist=havdist(evit->second->loc.lat,evit->second->loc.lon,g->second->loc.lat,g->second->loc.lon);
						dist=dist*1000;
						//cout<<"dist "<<evit->second->id_user<<" "<< g->second->id_user<<" "<<dist<<endl;

						if (dist < WDdistance){
							numsamego++;

							currevent->distgo=dist;
							currevent->status="member";
							currevent->idgo=idgo;
							int n=evit->first;
							//cout<<"eliminato"<<n<<endl;

							continue;
						}

						sgo.members--;
						smM[idgo]=sgo;
					}


				}


			currevent->idgo=0;
			currevent->status="alone";
			smem.idgo=0;
			smem.members=0;
			smM[evit->first]=smem;
			eves2.insert(pair <int,vector<Event>::iterator >(evit->first,evit->second));

			}

	}

	map<int,int> printduration;
	map<int,int> printdata;
	map<int,float> printmems;



	for(map <int,vector<Event>::iterator >::iterator evit2 = eves.begin(); evit2 != eves.end(); ++evit2){



		//cout<<"B "<<evit2->first<<" "<<endl;

		vector<Event>::iterator currevent=evit2->second;
		Smartphone smtemp2=smM[evit2->first];




		if(smtemp2.idgo==-1){

			if(smtemp2.members<1){

				//cout<<"t"<< evit2->first<<" data "<<dataPerGroup[evit2->first]<<" memb "<<smtemp2.members<<endl;
				//gruppo distrutto
				printduration[evit2->first]=durationGroup[evit2->first];
				//printduration[evit->first]=durationGroup[evit->first];
				printdata[evit2->first]=dataPerGroup[evit2->first];
				printmems[evit2->first]=groupmems[evit2->first];
				//stopfile<<" Data2 "<<dataPerGroup[evit->first]<<" duration "<<durationGroup[evit->first]<<endl;
				dataPerGroup[evit2->first]=0;
				durationGroup[evit2->first]=1;
				groupmems[evit2->first]=0;
				currevent->status="alone";
				currevent->idgo=0;
				smtemp2.idgo=0;
				smtemp2.members=0;
				smM[evit2->first]=smtemp2;
				eves2.insert(pair <int,vector<Event>::iterator >(evit2->first,evit2->second));
				//cout<<"rimasto"<<evit2->first<<" "<< smtemp2.members<<endl;
			}
			else{
				durationGroup[evit2->first]++;


				currevent->idgo=-1;
				currevent->status="go";
				currevent->members=smtemp2.members;
				//cout<<"Gdel"<<evit2->first<<" "<<endl;
				mapmember[currevent->id_user]=smtemp2.members;



			}
		}

	}



	for(map <int,vector<Event>::iterator >::iterator evit = eves2.begin(); evit != eves2.end(); ++evit){


		vector<int> idneighbor;
		vector<Event>::iterator currevent=evit->second;
		smtemp=smM[evit->first];


		//cout<<"C "<<evit->first<<" "<<endl;


		if(smtemp.idgo==-1){
			//cout<<evit->first<<" AOOOOOO "<<smtemp.id_user<<" "<<currevent->idgo<<endl;
			exit(-1);
		}
		int C=0;
		for(int i=-1;i<2;i++)
					for(int j=-1;j<2;j++){

					set<int> aroundid=members[(currevent->quad)+(i*numcolumns)+j];
					for(set<int>::iterator sit = aroundid.begin(); sit != aroundid.end(); ++sit){

						int CKT;
						sm2=smM[(*sit)];

						map<int, vector<Event>::iterator >::iterator itsecondev=eves2.find((*sit));
						if(itsecondev!=eves2.end())
						{
							vector<Event>::iterator secondev=itsecondev->second;


							CKT=computeCkt(currevent->loc,smtemp.smp,secondev->loc,sm2.smp,currevent->speed,secondev->speed);
							//CKT=computeCKT(currevent->speed,secondev->speed,currevent->loc,secondev->loc,smtemp.smp);
							if(CKT==0)
								C++;

							if(CKT!=-1){
								idneighbor.push_back((*sit));
							}


						}
					}

				}

		//int goai=updateGoaiDYN(C,smtemp.battery,5);
		int goai=updateGoaiDYN(C,smtemp.id_user,currevent->rssi);
		smtemp.goai=goai;
		currevent->gset.insert(Goaielement(goai,currevent->id_user));

		for (vector<int>::iterator itneigh = idneighbor.begin(); itneigh != idneighbor.end(); ++itneigh){
			vector<Event>::iterator evneg=eves2[(*itneigh)];
			evneg->gset.insert(Goaielement(goai,currevent->id_user));
		}


		smM[evit->first]=smtemp;
	}



//	for(set<int>::iterator itgo = gofinished.begin();itgo != gofinished.end() ; ++itgo ){
//
//		printduration[(*itgo)]=durationGroup[(*itgo)];
//		printdata[(*itgo)]=dataPerGroup[(*itgo)];
//		dataPerGroup[(*itgo)]=0;
//		durationGroup[(*itgo)]=1;
//	}



	for(map <int,int>::iterator itm = printduration.begin(); itm != printduration.end(); ++itm){

		stopfile<<" Data2 "<<printdata[itm->first]<<" duration "<<itm->second<<" membersAVG "<<printmems[itm->first]<<endl;

	}

//	gofinished.clear();




	for(map <int,vector<Event>::iterator >::iterator evit = eves2.begin(); evit != eves2.end(); ++evit){
		set<Goaielement>::iterator templast;

		vector<Event>::iterator currevent=evit->second;

		if(currevent->idgo==0){

			templast=currevent->gset.end();
			if(templast!=currevent->gset.begin()){
				templast--;

				currevent->idgo=GoElection(templast,eves2,currevent);
			}
			else{
				currevent->idgo=-2;
				currevent->status="alone";
			}
		}



		if(currevent->idgo==-1){

			dataPerGroup[currevent->id_user]=0;
			durationGroup[currevent->id_user]=1;
			groupmems[currevent->id_user]=0;
		}





	}
	for(map <int,vector<Event>::iterator >::iterator evit = eves2.begin(); evit != eves2.end(); ++evit){
		Smartphone smtemp3=smM[evit->first];

		if(evit->second->idgo==-1 and evit->second->members==0){
			smtemp3.members=0;
			smtemp3.idgo=-2;
			evit->second->idgo=-2;


		}
		else{
			smtemp3.members=evit->second->members;
			smtemp3.idgo=evit->second->idgo;
			mapmember[evit->second->id_user]=evit->second->members;
		}

		smM[evit->first]=smtemp3;

		if(evit->second->idgo>0){
			float distg;
			vector<Event>::iterator goevent=eves2[evit->second->idgo];

			distg=havdist(goevent->loc.lat,goevent->loc.lon,evit->second->loc.lat,evit->second->loc.lon);
			distg=distg*1000;
			evit->second->distgo=distg;

		}


	}
//
//			set<int>::iterator check= mit->second.find(g->second);
//			if( check !=mit->second.end()){
//				numsamego++;
//				continue;
//			}
//			else{
//				gomap.erase(mit->first);
//				newelection=true;
//			}
//
//
//
//		}
//		else{
//			newelection=true;
//		}
//
//
//
//		if(newelection==true){
//			numchanges++;
//			int goaimax=0;
//			int idmaxg=-1;
//			for(set<int>::iterator sit = mit->second.begin(); sit != mit->second.end(); ++sit){
//				//it_smM=smM.find((*sit));
//				map <int,vector<Event>::iterator >::iterator tev=eves.find((*sit));
//				tev->second->status="member";
//				//smM[(*sit)].goai=15;
//				//cout<<mit->first<<"  "<<((*tev).second.quad) <<"   "<<grid[mit->first].first.lat<<"   "<<grid[mit->first].second.lat<<" "<< (*tev).second.loc.lat<<endl;
//				int goai_temp= updateGoai(smM[(*sit)].battery,tev->second->loc,smM[(*sit)].smp,5,tev->second->speed,grid[mit->first].first,grid[mit->first].second);
//				smM[(*sit)].goai=goai_temp;
//				if(goai_temp>goaimax){
//					goaimax=goai_temp;
//					idmaxg=(*sit);
//				}
//
//			}
//			if(idmaxg!=1){
//				map <int,vector<Event>::iterator >::iterator tev2=eves.find(idmaxg);
//				gomap[mit->first]=idmaxg;
//				tev2->second->status="go";
//			}
//
//
//
//		}
//


	sort (evl.begin(), evl.end(), idgocomp);

	consumptionGo.clear();
	dataGoMin.clear();

	for (vector<Event>::iterator it_eventsL = evl.begin(); it_eventsL != evl.end(); ++it_eventsL){








		 if(ref_user!=(*it_eventsL).id_user){ //da mantenere
			 if(ref_user==-1){
				 ref_user=1;
			 }

			else{

				Smartphone temp_str(ref_user,sm.startBattery,old_battery,cap,"Outdoor",temp_dtbdelM,temp_dtsM,temp_s,sm.stop,route,generate,gen,flagstart,numgeneration,duration,sm.minwalk,sm.goai,timeroute,smM[ref_user].idgo,smM[ref_user].members);
				smM.erase(ref_user);
				smM.insert(pair <int, Smartphone >(ref_user,temp_str) );

				temp_s.clear();
				temp_dtsM.clear();
				temp_dtbdelM.clear();
				locstop=make_pair(0,0);

			}
		 ref_user=(*it_eventsL).id_user;

		 it_smM=smM.find(ref_user);
		 it_usersM=usersM.find(ref_user);
		 sm=(*it_smM).second;
		 temp_dtbdelM=sm.dtbd;
		 temp_dtsM=sm.dts;

		 temp_s=sm.smp;
		 old_battery=sm.battery;
		 cap=sm.capacity;
		 generate=sm.generate;
		 gen=sm.gen;
		 duration=sm.duration;
		 numgeneration=sm.numgeneration;
		 flagstart=sm.flagstart;
		 route=sm.route;
		 timeroute=sm.timeroute;

		 int ref_smartphone=sm.id_user;
		 string ref_context=sm.context;

		 }


		 ref_lat=(*it_eventsL).loc.lat;
		 ref_lon=(*it_eventsL).loc.lon;
		 float ref_alt=(*it_eventsL).loc.alt;









		 tm prev_timestamp;
		 Samples::iterator it_tmpsample;

		 it_tmpsample= --temp_s.end();





		 if(currentDay!=(*it_eventsL).timestamp.tm_wday){
		 				 int userusati=ref_user-counteruser;
		 				 results.push_back(datatotali/userusati);
		 				 results.push_back(cons_sensing/userusati);
		 				 results.push_back(cons_report/userusati);
		 				 cons_report=0;
		 				 cons_sensing=0;
		 				 datatotali=0;

		 				 counteruser=ref_user;
		 			 }
		 //inserted to avoid for
		 currentDay=(*it_eventsL).timestamp.tm_wday;

		 prev_timestamp=(*it_tmpsample).pos.timestamp;
		 //serve???
		 //prev_timestamp.tm_wday = currentDay;

		 // if sample list empty previous timestamp = actual event ts
		 if(temp_s.size() == 0){
			 //cout<<" ALLARME SAMPLES"<<endl;
			 exit(-1);
			 prev_timestamp=(*it_eventsL).timestamp;
		 }

		 // edited first event of the day has to be 0 as difference of minutes between samples
		 int time_between_samples = 0;
		 if((*it_eventsL).timestamp.tm_wday == prev_timestamp.tm_wday){
			 if((*it_eventsL).timestamp.tm_hour == prev_timestamp.tm_hour){
				 time_between_samples = abs((*it_eventsL).timestamp.tm_min-prev_timestamp.tm_min);
			 }


			 if((*it_eventsL).timestamp.tm_hour == prev_timestamp.tm_hour+1){
				 time_between_samples = ((*it_eventsL).timestamp.tm_min+(60 - prev_timestamp.tm_min));
			 }


			 //non dovrebbe entrarci mai?
			 if((*it_eventsL).timestamp.tm_hour > prev_timestamp.tm_hour+1){
				 int hours_counter2 = (*it_eventsL).timestamp.tm_hour - prev_timestamp.tm_hour ;
				 time_between_samples = hours_counter2*60+abs(((*it_eventsL).timestamp.tm_min-prev_timestamp.tm_min));
			 }
		 }

		 /*** GENERATION OF NEW SAMPLES ***/
		 if(flagstart==true){
			 //cout<<" ALLARME flagstart"<<endl;
			  exit(-1);
			 time_between_samples=0;
		 }
		 else{
			 time_between_samples=1;
		 }




		sm.minwalk=sm.minwalk + time_between_samples;






		if(it_eventsL->idgo>0 and dataGoMin[it_eventsL->idgo]>MaxDataGroup){

			//cout<<"Taglio "<<it_eventsL->id_user<<"  "<<it_eventsL->idgo<<endl;

			if(sm.minwalk==(*it_usersM).second.minutes){

				 Smartphone gosm=smM[it_eventsL->idgo];
				 gosm.members--;
				 smM[it_eventsL->idgo]=gosm;
			}

			mapmember[it_eventsL->idgo]--;
			it_eventsL->idgo=-2;

		}
		numtot++;

		if(it_eventsL->idgo==-2)
			numalone++;

//		if(it_eventsL->idgo==-1 and it_eventsL->members<2){
//
//			numalone++;
//		}

		if(it_eventsL->idgo==-1 ){



			if(mapmember[it_eventsL->id_user]!=it_eventsL->members){
				it_eventsL->members=mapmember[it_eventsL->id_user];
				//cout<<"memeber tagliato "<<it_eventsL->id_user<<endl;

			}

			if(it_eventsL->members>30){
				//cout<<"Error "<<it_eventsL->id_user<<" dur "<<durationGroup[it_eventsL->id_user]<< " members "<< it_eventsL->members<<" min "<<it_eventsL->timestamp.tm_min<<" cons "<<mapmember[it_eventsL->id_user]<<" "<<dataGoMin[it_eventsL->id_user]<<endl;

				exit(-2);
			}
			//cout<<"GO "<<it_eventsL->id_user<<" dur  "<<durationGroup[it_eventsL->id_user]<< " members "<< it_eventsL->members<<" min "<<it_eventsL->timestamp.tm_min<<" cons "<<consumptionGo[it_eventsL->id_user]<<" "<<dataGoMin[it_eventsL->id_user]<<endl;

			groupmems[ref_user]+=it_eventsL->members;
			membersfile<<it_eventsL->id_user<<" "<<it_eventsL->members<<endl;
			numgos++;

		}





		 // Always on sensors: consider sampling frequency, take time between previous timestamp and current
		 // and generate a number of samples accordingly

		 float temp_value=fRand(1.0,5.0);
		 float temp_size=fRand(500.0, 2000.0); // Kilobyte
		 string temp_type="GPS";

		 // GPS
		 Location temp_loc(ref_lat,ref_lon,ref_alt);
		 Position temp_p(temp_loc,(*it_eventsL).timestamp);
		 Sample temp_sample(temp_type,temp_value,temp_size,false,temp_p);
		 temp_s.push_back(temp_sample);


			 float tau_tx=lambda_g*(0.000192+((28.0+1500.0)/wifi_uplink_data_tx));// 192 us: PLCP time + (Header+Payload)/data rate
			 float wifi_power=rho_id+rho_tx*tau_tx+lambda_g*gamma_g;// power to tx one packet

		// * * ACCELLEROMETER
			 int acc_num_samples_to_generate=acc_sample_frequency*60*time_between_samples;
			 int acc_data_size=(acc_num_samples_to_generate*acc_sample_size)/8;// to have Bytes
			 int acc_energy_sampling=acc_current*(60*time_between_samples);// these are uAs (remember: 3600 uAs= 1uAh)
			 float acc_energy_sampling_hour=acc_energy_sampling/3600000.00;// mAh

		int acc_packets=(int)acc_data_size/1500;
		float acc_tx_time=(acc_data_size*8.0)/wifi_uplink_data_tx;// in seconds (bit / bps)

		float acc_wifi_energy=wifi_power*acc_packets; // Joules (Watt*seconds)
		float acc_wifi_consumption=( acc_wifi_energy/( 1000*3.6 ) )*( 1000/volt ); // mah

		outputsimdata << currentDay << " "<<(*it_eventsL).timestamp.tm_hour << " "
					  << (*it_eventsL).timestamp.tm_min << " "
					  << sm.id_user << " "
					  << ref_lat << " "// position latitude
					  << ref_lon << " "// position longitude
					  << sm.battery << " "
					  << "Accelerometer" << " "
					  << acc_num_samples_to_generate << " "
					  << acc_data_size << " "// in Bytes
					  << acc_packets << " "// number of packets
					  << acc_tx_time << " "// uplink transmission time in seconds
					  << acc_energy_sampling_hour << " "// in uAh
					  << acc_wifi_energy//
					  << endl;

		// * * TEMPERATURE
		int gps_num_samples_to_generate=gps_sample_frequency*60*time_between_samples;
		int gps_data_size=(gps_num_samples_to_generate*gps_sample_size)/8;// to have Bytes
		int gps_energy_sampling=gps_current*(60*time_between_samples)*gps_sample_frequency;// these are uAs (remember: 3600 uAs= 1uAh)
		float gps_energy_sampling_hour=gps_energy_sampling/3600000.00;// uAh

		int gps_packets=(int)gps_data_size/1500;
		float gps_tx_time=(gps_data_size*8.0)/wifi_uplink_data_tx;// in seconds (Byte / Bps)
		float gps_wifi_energy=wifi_power*gps_packets; // Joules (Watt*seconds)

		float gps_wifi_consumption=( gps_wifi_energy/( 1000*3.6 ) )*( 1000/volt );

		outputsimdata << currentDay << " " << (*it_eventsL).timestamp.tm_hour << " "
					  << (*it_eventsL).timestamp.tm_min << " "
					  << sm.id_user << " "
					  << ref_lat << " "// position latitude
					  << ref_lon << " "// position longitude
					  << sm.battery << " "
					  << "GPS" << " "
					  << gps_num_samples_to_generate << " "
					  << gps_data_size << " "// in Bytes
					  << gps_packets << " "
					  << gps_tx_time << " "// uplink transmission time in seconds
					  << gps_energy_sampling_hour << " "// in mAh
					  << gps_wifi_energy
					  << endl;

		// * * PRESSURE
		int prox_num_samples_to_generate=prox_sample_frequency*60*time_between_samples;
		int prox_data_size=(prox_num_samples_to_generate*prox_sample_size)/8;// to have Bytes
		int prox_energy_sampling=prox_current*(60*time_between_samples);// these are uAs (remember: 3600 uAs= 1uAh)
		float prox_energy_sampling_hour=prox_energy_sampling/3600000.00;// mAh

		int prox_packets=(int)prox_data_size/1500;
		float prox_tx_time=(prox_data_size*8.0)/wifi_uplink_data_tx;// in seconds (Byte / Bps)
		float prox_wifi_energy=wifi_power*prox_packets; // Joules (Watt*seconds)
		float prox_wifi_consumption=( prox_wifi_energy/( 1000*3.6 ) )*( 1000/volt );

		outputsimdata << currentDay << " " << (*it_eventsL).timestamp.tm_hour << " "
					  << (*it_eventsL).timestamp.tm_min << " "
					  << sm.id_user << " "
					  << ref_lat << " "// position latitude
					  << ref_lon << " "// position longitude
					  << sm.battery << " "
					  << "Proximity" << " "
					  << prox_num_samples_to_generate << " "
					  << prox_data_size << " "// in Bytes
					  << prox_packets << " "
					  << prox_tx_time << " "//uplink transmission time in seconds
					  << prox_energy_sampling_hour << " "// in mAh
					  << prox_wifi_energy
					  << endl;


		 temp_dtbdelM["Accelerometer"]+=acc_data_size; // number of Bytes
		 temp_dtbdelM["GPS"]+=1.0;
		 float tot_data=0;

		 float cons_sens=0;
		 float cons_wifi=0;

		 if(sm.stop==0){
			 tot_data=acc_data_size+prox_data_size+gps_data_size;
			 temp_dtbdelM["data"]+=tot_data;
			 cons_sens=prox_energy_sampling_hour+gps_energy_sampling_hour+acc_energy_sampling_hour;
			 temp_dtbdelM["consumption-data"]+=cons_sens;

			 int packs=(int)tot_data/1500;
			 packs+=1;
			 if(typeCons==0)
				 packtotali +=packs;
			 float tot_wifi_energy=wifi_power*packs; // Joules (Watt*seconds)
			 cons_wifi=( tot_wifi_energy/( 1000*3.6 ) )*( 1000/volt );
			 temp_dtbdelM["consumption-wifi"]+=cons_wifi;
			 temp_dtbdelM["Energy"]+=cons_sens;
		}
	/*			 cout<< acc_data_size<<" " << prox_data_size<<" "<< gps_data_size<<endl;

		 cout<<"DAI    " <<cons_wifi<<endl;*/



			 enTot= cons[typeCons]*temp_dtbdelM["Minutes"];

			 if(traces==false){
				 enTot=temp_dtbdelM["consumption-data"];
				 if(typeCons==0){
					 enTot+=temp_dtbdelM["consumption-wifi"];
					 temp_dtbdelM["Energy"]+=cons_wifi;
				 }
					 }

			 float enmax= (cap/100)*bat_threshold;




			 if(pda==true){
				 float si=datasent/dataexpected;
				 if(b>0 ){
					 if(si>upbound){
						 if(b>1)
							 b=b-1;

						 else
							 b=b/(1+b);

					 }
				 }
				if(b<50 ){
					 if(si<lowbound){
						 if(b<1)
							 b=b/(1-b);
						 else
							 b=b+1;
					 }
				 }
				 b=0.5;
				 threshold=1-pow(si,b);
				 tres<< threshold<<" "<<datasent<<" "<<b<<" "<<si<<" "<<pow(si,b) <<endl;
			 }





			 if(typeCons==2){
				 float prob=(float(rand()) /float(RAND_MAX));
				 if(threshold<prob)
					 gen=false;
				 else
					 gen=true;

			 }

			 if(typeCons==3 && duration==0){
				 float prob=(float(rand()) /float(RAND_MAX));
				 if(percentages[((*it_eventsL).timestamp.tm_hour)]<prob)
					 gen=false;
				 else{
					 duration=duration_hour[((*it_eventsL).timestamp.tm_hour)];
					 gen=true;
				 }






			 }

			 if(duration>0)
				 duration-=time_between_samples;

			 if(typeCons==1)
				 gen=true;

			 if(typeCons==0){

				 if(enTot>=enmax && sm.stop==0){

					 sm.stop=1;


					 sb= sm.startBattery;
	/*					 if(typeCons==1){
						 ctx=temp_dtbdelM["Minutes"] * reporting_cost;

						 enTot+=ctx;



					 }*/

					old_battery=sm.battery;
					float rem= (cap/100)*old_battery;
					rem = rem - enTot;

					old_battery= (rem/cap)*100;
					locstop=make_pair(ref_lon,ref_lat);

					 //cout<< "Stopped"<<" "<<ref_user<<endl;
					gen=false;
				 }
				 if(sm.stop==0)
					 temp_dtbdelM["Minutes"]+=time_between_samples;
			 }


		float temp_data=0.0;
		 if(gen || typeCons!=0){
			 if(traces==true)
			 if(typeCons!=0)
			 temp_dtbdelM["Minutes"]+=time_between_samples;

			 temp_dtbdelM["Total"]+=(time_between_samples * bytes_per_minute);
			 temp_data=tot_data;
			 if((typeCons==2 || typeCons==3) && gen ){
				 if(traces==true){
					 if(temp_dtbdelM["Total"]<kbpermin){
						 temp_dtsM["Total"]+=temp_dtbdelM["Total"];
						 datasent+=temp_dtbdelM["Total"];
						 temp_dtbdelM["Total"]=0;

					 }
					 else{
						 temp_dtsM["Total"]+=kbpermin;
						 temp_dtbdelM["Total"]-=kbpermin;
						 datasent+=kbpermin;
					 }
				 }
				 else{
					 temp_dtbdelM["Minutes"]+=time_between_samples;

					 if(temp_dtbdelM["data"]<kbpermin){

						 temp_dtsM["Total"]+=temp_dtbdelM["data"];
						 temp_data=temp_dtbdelM["data"];
						 int tot_packets=(int)temp_dtbdelM["data"]/1500; //Bytes/dimension of one packet
						 tot_packets+=1;
						 float wifitemp=((tot_packets*wifi_power)/( 1000*3.6 ) )*( 1000/volt);
						 temp_dtbdelM["Energy"]+=wifitemp;
						 packtotali+=tot_packets;
						 temp_dtbdelM["data"]=0;


					 }

					 else{

						 temp_dtsM["Total"]+=kbpermin;
						 temp_dtbdelM["data"]-=kbpermin;
						 temp_data=kbpermin;
						 int tot_packets=(int)kbpermin/1500;//Bytes/dimension of one packet
						 tot_packets+=1;
						 packtotali+=tot_packets;
						 temp_dtbdelM["Energy"]+=((tot_packets*wifi_power)/( 1000*3.6 ) )*( 1000/volt);

					 }



				 }



			 }
		 }
	//			 if(ref_user==42)
	//				 cout<< ref_lon<<" "<<ref_lat<<endl;
		 route.push_back(make_pair(ref_lon,ref_lat));
		 timeroute.push_back((*it_eventsL).timestamp.tm_min);

		 generate.push_back(gen);
		 if(gen==true && flagstart==false)
			 numgeneration+=1;





		 if(flagstart)
			 flagstart=false;
		 float datiTemp=0;
		 if(traces==false){


				datiTemp= temp_data/1000;    // from Byte to KB
				//enTot=temp_dtbdelM["consumption-wifi"]+ temp_dtbdelM["consumption-data"];
				//cons_sensing+=temp_dtbdelM["consumption-data"];
				//cons_report+=temp_dtbdelM["consumption-wifi"];

				if(typeCons==2 || typeCons==3){
					datiTemp=temp_data/1000;
					//enTot=temp_dtbdelM["Energy"];
					//cons_sensing+=temp_dtbdelM["consumption-data"];
					//cons_report+=enTot-temp_dtbdelM["consumption-data"];

				}
				if((it_eventsL->idgo)>0){
					dataPerGroup[it_eventsL->idgo]= dataPerGroup[it_eventsL->idgo] +datiTemp;
				}

			}





			 float upwd_perbit,uplte_perbit,downwd_perbit,thrwd,thrlte;

			 thrlte=it_eventsL->rssi;

			 //cout<< uplte_perbit <<" "<< upwd_perbit<<" " <<tot_data*8*downwd_perbit<<" "<< tot_wifi_energy <<endl;


			 if(it_eventsL->idgo>0){


				 float dt=it_eventsL->distgo;

				 for(int ij=0;ij<5;ij++){
					 if(dt<distthr[ij]){
						 thrwd=thr[ij];
						 break;
					 }
				 }

				upwd_perbit = (283.17* thrwd + 132.86)/(thrwd*wifi_uplink_data_tx*1000);


				//uplte_perbit = (438.39* thrwd + 1288.04)/(thrlte*wifi_uplink_data_tx*1000);

				downwd_perbit= (137.01* thrwd + 132.86)/(thrwd*wifi_uplink_data_tx*1000);


				consumptionGo[it_eventsL->idgo]= consumptionGo[it_eventsL->idgo] + (1000*8*datiTemp*downwd_perbit);

				dataGoMin[it_eventsL->idgo]=  dataGoMin[it_eventsL->idgo] + datiTemp;

				temp_dtbdelM["wdcons"]+= 1000*8*datiTemp*upwd_perbit;


				//cout<<"member "<<it_eventsL->id_user<<"  GO "<<it_eventsL->idgo<<" data "<< dataPerGroup[it_eventsL->idgo]<<" min "<<it_eventsL->timestamp.tm_min<<" dist "<<it_eventsL->distgo<<" dati "<<datiTemp<<" membs "<<mapmember[it_eventsL->idgo]<<endl;
				nummember++;

				memconsfile<<1000*8*datiTemp*upwd_perbit<<endl;
				temp_dtbdelM["member"]+=1;
			 }



				if(it_eventsL->idgo==-1){
					    uplte_perbit = (438.39* thrlte + 1288.04)/(thrlte*wifi_uplink_data_tx*1000);

					    temp_dtbdelM["wdcons"]+=consumptionGo[it_eventsL->id_user];
					    temp_dtbdelM["wdcons"]+=uplte_perbit*1000*8*(datiTemp + dataGoMin[it_eventsL->id_user]);
					    goconsfile<<consumptionGo[it_eventsL->id_user]+(uplte_perbit*1000*8*(datiTemp + dataGoMin[it_eventsL->id_user]))<<endl;
					    temp_dtbdelM["go"]+=1;
					    temp_dtbdelM["nummember"]+=it_eventsL->members;
				}

				if(it_eventsL->idgo==-2){
						uplte_perbit = (438.39* thrlte + 1288.04)/(thrlte*wifi_uplink_data_tx*1000);
						temp_dtbdelM["wdcons"]+=NoNoGroup*uplte_perbit*(1000*8*datiTemp);
						ngconsfile<<uplte_perbit*(1000*8*datiTemp)<<endl;

						temp_dtbdelM["alone"]+=1;
				}







//		 if(it_eventsL->idgo>0)
//			 cout<<"DAta"<<dataPerGroup[it_eventsL->idgo]<<endl;




			 if(sm.minwalk==(*it_usersM).second.minutes){


			 if(it_eventsL->idgo==-1 ){

					stopfile<<"Data1 "<<dataPerGroup[ref_user]<<" duration "<<durationGroup[ref_user]<<" membsAVG "<< groupmems[ref_user]<<endl;


					dataPerGroup[ref_user]=0;
					durationGroup[ref_user]=1;
					groupmems[ref_user]=0;


			 }

			 if(it_eventsL->idgo>0 ){

				 //cout<<" Finitooo "<<endl;
				 Smartphone membtempsmt=smM[it_eventsL->idgo];
				 membtempsmt.members--;
				 smM[it_eventsL->idgo]=membtempsmt;

			 }



			 enTot=0;

			 float dati;
			 sb=sm.startBattery;

			 if(typeCons==0 || typeCons==1)
				 dati=temp_dtbdelM["Total"];

			 if(typeCons==2 || typeCons==3)
				 dati=temp_dtsM["Total"];


			 if(typeCons==1 ){

				ctx=dati * reporting_cost;
				enTot+=ctx;
			}

			if(typeCons==3 || pda==true ){

				ctx=numgeneration * pbrep_cost;
				enTot+=ctx;
				enTot+= cons[3]*(temp_dtbdelM["Minutes"]-numgeneration);
				cons_sensing+=cons[3]*(temp_dtbdelM["Minutes"]-numgeneration);
				cons_report+=ctx;
			}
			else{
				enTot+= cons[typeCons]*temp_dtbdelM["Minutes"];
				cons_sensing+=cons[3]*temp_dtbdelM["Minutes"];
				cons_report+=enTot-cons[3]*temp_dtbdelM["Minutes"];
									}

			if(traces==false){

				dati= temp_dtbdelM["data"]/1000;    // from Byte to KB
				enTot=temp_dtbdelM["consumption-wifi"]+ temp_dtbdelM["consumption-data"];
				cons_sensing+=temp_dtbdelM["consumption-data"];
				cons_report+=temp_dtbdelM["consumption-wifi"];

				if(typeCons==2 || typeCons==3){
					dati=temp_dtsM["Total"]/1000;
					enTot=temp_dtbdelM["Energy"];
					cons_sensing+=temp_dtbdelM["consumption-data"];
					cons_report+=enTot-temp_dtbdelM["consumption-data"];

				}
			}




			float totalwdconsumption=0.0,nogroup_consumption;

			uplte_perbit = (438.39* thrlte + 1288.04)/(thrlte*wifi_uplink_data_tx*1000);
			totalwdconsumption=temp_dtbdelM["wdcons"];


			nogroup_consumption=uplte_perbit*1000*8*dati;

			wdconsfile<<it_eventsL->id_user<<" "<<nogroup_consumption<<" "<<totalwdconsumption<<endl;

			if(totalwdconsumption>2.7){
				ex_go+=(temp_dtbdelM["go"]/sm.minwalk)*100;
				ex_ng+=(temp_dtbdelM["alone"]/sm.minwalk)*100;
				ex_mem+=(temp_dtbdelM["member"]/sm.minwalk)*100;
				ex_nummem+=temp_dtbdelM["nummember"]/temp_dtbdelM["go"];
				ex_tot+=1;

				}
								//exceedfile<<(temp_dtbdelM["alone"]/sm.minwalk)*100<<" "<<(temp_dtbdelM["go"]/sm.minwalk)*100<<" "<<(temp_dtbdelM["member"]/sm.minwalk)*100<<" "<<temp_dtbdelM["nummember"]/temp_dtbdelM["go"]<<endl;
			else{


				no_go+=(temp_dtbdelM["go"]/sm.minwalk)*100;
				no_ng+=(temp_dtbdelM["alone"]/sm.minwalk)*100;
				no_mem+=(temp_dtbdelM["member"]/sm.minwalk)*100;
				if(temp_dtbdelM["go"]!=0)
					no_nummem+= temp_dtbdelM["nummember"] /temp_dtbdelM["go"];
				no_tot+=1;

			}
			old_battery=sm.battery;
			float rem= (cap/100)*old_battery;
			rem = rem - enTot;

			old_battery= (rem/cap)*100;
			smM.erase(ref_user);
			Smartphone temp_str(ref_user,sb,old_battery,cap,"Outdoor",temp_dtbdelM,temp_dtsM,temp_s,sm.stop,route,generate,gen,flagstart,numgeneration,duration,sm.minwalk,sm.goai,timeroute,sm.idgo,sm.members);
			smM.insert(pair <int, Smartphone >(ref_user,temp_str) );

			if(locstop.first==0){
				locstop=make_pair(ref_lon,ref_lat);
			}
			datatotali+=dati;
			datatotalitotgiorni+=dati;

			float mintotali=(*it_usersM).second.minutes;
			if(mintotali!=0){
			float contrib=numgeneration/mintotali;
			float batteryrel= enTot/(rem + enTot);
			batcontrib+=batteryrel;
			quadbatcontrib+=batteryrel*batteryrel;
			totcontribution +=contrib;
			quadcontribution+=contrib*contrib;
			}
			activefile << sm.id_user <<" "<< dati<<" "<<enTot <<" "<< sb-old_battery <<" "<<old_battery << " "<<numgeneration << " "<< (*(it_eventsL)).timestamp.tm_hour<<","<<(*(it_eventsL)).timestamp.tm_min<< " "<<locstop.first<<" "<<locstop.second <<" "<< temp_dtbdelM["consumption-wifi"]<< " "<<(*it_usersM).second.minutes << endl;

			int k=0;
			//if(ref_user==23 || ref_user==85||ref_user==119 ||ref_user==53 ||ref_user==106 )
			float datigenpermin=dati/numgeneration;
			float dg=0.0;
			if((*(it_eventsL)).timestamp.tm_hour==12 && (*(it_eventsL)).timestamp.tm_wday==0)
			for(int i=0;i<(route.size()-1);i++){
				dg=0.0;
				if(generate[i+1]==true){
					dg=datigenpermin;
					routesfile <<timeroute[i]<<"	"<<route[i].first<<" "<<route[i].second<< "	"<< generate[i+1] <<"	"<<dg<< endl;
				}
			}


		 }


	}


 }

		 rolefile3<<"3 "<<((float)numalone/(float)numtot)*100<<" "<<((float)numgos/(float)numtot)*100<<" "<<((float)nummember/(float)numtot)*100<<endl;
		 rolefile<<"3 "<<((float)numalone/(float)numtot)*100<<" "<<((float)numgos/(float)numtot)*100<<" "<<((float)nummember/(float)numtot)*100<<endl;

		 cout <<"DATA Generated   "<<datatotali<<endl;
		 fairfile <<"tot Contribution  "<<totcontribution<<" quad contrib  "<<quadcontribution<<" Fairness  "<<((totcontribution*totcontribution)/quadcontribution)/num_users<<endl;

		 batfairfile <<"BATTERY CONTRIBUTION  "<<batcontrib<<" quad contrib  "<<quadbatcontrib<<" Fairness  "<<((batcontrib*batcontrib)/quadbatcontrib)/num_users<<endl;

		 cout <<"cambiamenti  "<<numchanges<<" stesso go  "<<numsamego<<endl;
		 exceedfile<<"3 "<<ex_ng/ex_tot<<" "<<ex_mem/ex_tot<<" "<<ex_go/ex_tot<<" "<<ex_nummem/ex_tot<<endl;
		 noexceedfile<<"3 "<<no_ng/no_tot<<" "<<no_mem/no_tot<<" "<<no_go/no_tot<<" "<<no_nummem/no_tot<<endl;

		 int userusati2=ref_user-counteruser;
		 results.push_back(datatotali/userusati2);
		 results.push_back(cons_sensing/userusati2);
		 results.push_back(cons_report/userusati2);
		 cons_report=0;
		 cons_sensing=0;
		 datatotali=0;

		counteruser=ref_user;

		perc.close();
	    outputsimdata.close();
	    stopfile.close();
	    activefile.close();

	    return results;




}





















