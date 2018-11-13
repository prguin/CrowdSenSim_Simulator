/*
 * Event.cc
 *
 *
 *  Created on: 30 giu 2016
 *      Author: Giuseppe Cacciatore
 */

#include "../Headers/Event.h"

#include <iostream>
#include <string>
#include <fstream>

#include "../Headers/Antenna.h"
#include "../Headers/ClockManagement.h"
#include "../Headers/ReadValues.h"



Events eventsLFunc;

Events::iterator it_eventsLFunc;
map <time_t,vector<Event> > contacts;




bool eventComparator(const Event& lhs, const Event& rhs){
/**
 * It makes a time comparison between events, and return a boolean value
 */
double seconds;
struct tm l,r;
l=lhs.timestamp;
r=rhs.timestamp;



if(l.tm_wday==r.tm_wday){

	seconds=difftime(mktime(&l),mktime(&r));


	if(seconds >0)
		return false;

	 if(seconds <0)
		return true;

	 if(seconds==0){
		if(lhs.id_user<rhs.id_user)
			return true;
		else
			return false;
	 }
}
else
	if(l.tm_wday < r.tm_wday )
		return true;
	else
		return false;



}










map <time_t,vector<Event> > readListOfEvents (string pname,int dec,Users* pusersM, int days, int nusrs,Events* eventsL2, map <int , pair<Location,Location> >* grid,map <int ,vector<Location> >* poicords,int* numcoluns,Smartphones *psmM){

int d;
/*
 * It reads the default list of events.
 * It happens if in the "Setup.txt" file we do not want to create a new list of events.
 * Automatically the simulation will be set for 2000 users in the city.
 *
 */
int num_event=0;

int id_user=0,id_current=1,minutes=-1,current_min=-1;
Users::iterator uscurr;
Users usersM2;
set<Goaielement> gmap;
for (d=0;d<days;d++){
	string fname;
	stringstream ss2;
	ss2 << pname << d << ".txt";
	fname=ss2.str();

	char const row_delim = '\n';
	typedef vector<vector<string> > Rows;
	Rows rows;

	time_t t;
	time(&t);
	srand(time(&t));
	tm ref_time;
	ref_time.tm_min=48;
	ref_time.tm_hour=12;
	t=mktime(&ref_time);

	cout << "**Reading list of events**" << endl;

	string dummyLine;
	ifstream eventinputfile(fname.c_str());

	getline(eventinputfile, dummyLine);
	for(string row; getline(eventinputfile, row, row_delim); ){

		num_event++;

		istringstream buf(row);
		istream_iterator<string> beg(buf), end;

		vector<string> tokens(beg, end);

		id_user=atoi(tokens[0].c_str());
		if(id_user!=id_current ){


			//usersM[id_current].id_pos=minutes;

			User temp_str(id_current,minutes);
			usersM2.insert(pair <int,User >(id_current,temp_str));
			if(id_current==nusrs)
				break;

			id_current=id_user;
			minutes=-1;
			current_min=-1;
		 }

		  Location temp_loc(atof(tokens[1].c_str()),atof(tokens[2].c_str()),atof(tokens[3].c_str()));
		  tm ref_time;
		  ref_time.tm_hour=atoi(tokens[5].c_str());
		  ref_time.tm_min=atoi(tokens[6].c_str());


		  if(current_min!=ref_time.tm_min){
			  minutes++;
			  current_min=ref_time.tm_min;
		  }

		  ref_time.tm_wday=atoi(tokens[4].c_str());

		  int numquad=atoi(tokens[7].c_str());
		  float speed=atof(tokens[8].c_str());

		  Smartphone smt=(*psmM)[id_user];
		  bool fstart=smt.flagstart;

		  if(fstart){

			  Position temp_p(temp_loc,ref_time);
			  Sample temp_sample("GPS",0,0,false,temp_p);
			  smt.smp.push_back(temp_sample);

			  smt.flagstart=false;

			  (*psmM)[id_user]=smt;

		  }
		  else{
		  int rssi= rand() % 4;
		  rssi++;
		  Event temp_event(id_user,temp_loc,ref_time,numquad,speed,"alone",-2,gmap,0,0,0,rssi);

		  eventsLFunc.push_back(temp_event);
		  time_t t=mktime(&ref_time);
		  ref_time.tm_min=atoi(tokens[6].c_str());
		  t=mktime(&ref_time);
		  contacts[t].push_back(temp_event);
		  //if((id_user)==1)
		  	//cout<<"ECCOlO "<<t<<"  "<<temp_loc.lat<<"   "<<temp_loc.lon<<" "<< ref_time.tm_min<<endl;
		  }
	}
	string c ("####");
	for(string row; getline(eventinputfile, row, row_delim); ){

		istringstream buf(row);
		istream_iterator<string> beg(buf), end;


		vector<string> tokens(beg, end);
		if(tokens[0].compare(c)== 0){
			break;
		}
		Location bot(0,0,0),up(0,0,0);

		bot.lon=atof(tokens[1].c_str());
		bot.lat=atof(tokens[2].c_str());
		up.lat=atof(tokens[4].c_str());
		up.lon=atof(tokens[3].c_str());
		int q=atoi(tokens[0].c_str());
		(*grid)[q]=(make_pair(bot,up));

	}
	int flagst=0;
	int idpoi=1;
	for(string row; getline(eventinputfile, row, row_delim); ){

		istringstream buf(row);
		istream_iterator<string> beg(buf), end;


		vector<string> tokens(beg, end);
		if(flagst==0){
			flagst=1;
			(*numcoluns)=atoi(tokens[0].c_str());
			continue;
		}

		Location p(0,0,0);

		p.lon=atof(tokens[1].c_str());
		p.lat=atof(tokens[2].c_str());
		int nqu=atoi(tokens[3].c_str());

		p.alt=(float)idpoi;
		idpoi++;
		//cout<< (int)p.alt<<"  "<<p.lon<<endl;
		for(int i=-1;i<2;i++)
			for(int j=-1;j<2;j++)
				(*poicords)[nqu+(i*(*numcoluns))+j].push_back(p);

	}

	if (!eventinputfile.eof()){
		  cerr << "Error in reading the file 'UserMobilityListEventsDefault"<< d <<".txt'!"<<endl;
	}
	eventinputfile.close();
}


(*eventsL2) =eventsLFunc;

cout << "**Reading ended**" << endl;


(*pusersM)=usersM2;




if(dec==1){




	int d,current_neigh,num_neigh,radius;
	map<time_t,map<int,set<int> > > durationcontacts;
	map<int,map<time_t,int> > usercontacts;
	vector<Event>::iterator check;
	map<int,set<int> > temp;
	time_t currenttime;
	radius=readRay();
	for (map <time_t,vector<Event> >::iterator it = contacts.begin(); it != contacts.end(); ++it){

		vector<Event> &internal_map = it->second;
		current_neigh=0;
		num_neigh=0;
		if(it!=contacts.begin())
			durationcontacts[currenttime]=temp;
			temp.clear();
		currenttime=it->first;

		for (vector<Event>::iterator it2 = internal_map.begin(); it2 != internal_map.end(); ++it2){
			check=it2;
			check++;
			current_neigh=0;

			while (check != internal_map.end()){

				if((*check).id_user==(*it2).id_user || current_neigh==(*check).id_user){
							check++;
							continue;
				}

				d=havdist((*check).loc.lat,(*check).loc.lon,(*it2).loc.lat,(*it2).loc.lon)*1000;
				if(d<radius){

					current_neigh=(*check).id_user;
					set<int> t=temp[current_neigh];
					if(t.insert((*it2).id_user).second){
						temp[current_neigh].insert((*it2).id_user);
						temp[(*it2).id_user].insert(current_neigh);

						usercontacts[(*it2).id_user][currenttime]++;
						usercontacts[current_neigh][currenttime]++;


					}

				}
				check++;


			}



		}

	}



	cout << "**Elaboration struct ended**" << endl;



	// Print Maps of Neighbour

//	for (map<time_t,map<int,set<int> > >::iterator it = durationcontacts.begin(); it != durationcontacts.end(); ++it){
//
//		eventordinati << endl;
//		eventordinati << it->first << " : ";
//		map<int,set<int> > &internal_map = it->second;
//
//		for (map<int,set<int> >::iterator it2 = internal_map.begin(); it2 != internal_map.end(); ++it2){
//			eventordinati << endl;
//			eventordinati << endl;
//
//			eventordinati << (*it2).first <<" vicino a :";
//
//			set<int>  internal_set = it2->second;
//
//			for (set<int>::iterator it3= internal_set.begin(); it3 != internal_set.end(); ++it3){
//				eventordinati <<" "<< (*it3);
//			}
//		 }
//	}


	ofstream stabfile;
	stabfile.open("./stab.txt");

	float ntot,nlost,nnew,nstab;
	map<int,set<int> >::iterator next;
	map<int,vector<float> > stabcontacts;
	map<int,set<int> > primotempo,secondotempo;
	set<int> tempset;

	for (map<time_t,map<int,set<int> > >::iterator it = durationcontacts.begin(); it != durationcontacts.end(); ++it){


		primotempo= it->second;
		it++;
		secondotempo=it->second;

		for (map<int,set<int> >::iterator it2 = primotempo.begin(); it2 != primotempo.end(); ++it2){
			ntot=0;
			nstab=0;

			nnew=0;
			ntot+=(*it2).second.size();
			nlost=ntot;
			next=secondotempo.find((*it2).first);
			if(next !=secondotempo.end()){
				ntot+=(*next).second.size();
				tempset=(*next).second;
				for (set<int>::iterator it3= (*it2).second.begin(); it3 != (*it2).second.end(); ++it3){
					if(tempset.erase((*it3))<1){
						nlost--;
					}
				}
				nnew=tempset.size();
				
			}
			
			nstab=(nnew+nlost)/ntot;
			if(nstab==0)
			//cout<< ntot<< nnew << nlost<< endl; 
			durationcontacts[it->first].erase(it2->first);
			stabcontacts[it2->first].push_back(nstab);


		 }
		it--;
	}

	cout << "**algorrithm stab ended**" << endl;


//print user contacts

//	for (map <int,map<time_t,int> >::iterator it = usercontacts.begin(); it != usercontacts.end(); ++it){
//		eventordinati << it->first << " : ";
//		map<time_t, int> &internal_map = it->second;
//		for (map<time_t, int>::iterator it2 = internal_map.begin(); it2 != internal_map.end(); ++it2){
//			if (it2 != internal_map.begin())
//				eventordinati << ",";
//			eventordinati << it2->first << ":" << it2->second;
//		}
//		eventordinati << endl;
//	}
//



	float totstab,sz,stabres;

	for (map <int,vector<float> >::iterator it = stabcontacts.begin(); it != stabcontacts.end(); ++it){
		totstab=0;
		vector<float> internal_map=it->second;

		for (vector<float>::iterator it2 = internal_map.begin(); it2 != internal_map.end(); ++it2){
			
			totstab+=(*it2);
		}
		sz=internal_map.size();
		if(sz==0){
			stabres=0;

		}
		else
			stabres=totstab/sz;

		stabfile<<  float(stabres) << endl;
	}

	cout << "**first print ended**" << endl;

	stabfile.close();

	ofstream contfile;
	contfile.open("./cont.txt");
	float totn;
	int mintot,idu;
	float res;
	for(idu=0;idu<nusrs;idu++){
	//for (map <int,map<time_t,int> >::iterator it = usercontacts.begin(); it != usercontacts.end(); ++it){
		totn=0;
		map <int,map<time_t,int> >::iterator it = usercontacts.find(idu);
		if(it != usercontacts.end() ){



			for (map<time_t, int>::iterator it2 = (*it).second.begin(); it2 != (*it).second.end(); ++it2){

				totn+=(*it2).second;
			}
		}
		uscurr=usersM2.find((*it).first);
		mintot=(*uscurr).second.minutes;
		if(mintot==0)
			res=totn;
		else
			res=totn/mintot;
//		eventordinati <<float(res) <<" " <<(*it).first<<endl;
		contfile <<float(res) <<endl;

	}

	contfile.close();

	usersM2.clear();
	durationcontacts.clear();
	usercontacts.clear();




	time_t t1 = time(0);   // get time now
	struct tm * now = localtime( & t1 );
	//cout << (now->tm_hour) << '-' << (now->tm_min) << '-' <<  now->tm_sec << endl;


}

return contacts;

}



