/*
 * Smartphones.h

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#ifndef SMARTPHONES_H_
#define SMARTPHONES_H_

#include "Sample.h"

/**
 * The amount of data to be delivered
 */
typedef std::map<string, float> DatatbdelM;

/**
 * The amount of data already sent
 */
typedef std::map<string, float> DatasentM;

using namespace std;

struct Smartphone{

	int id_user;                /**< User ID */
	float startBattery;         /**< Battery Level (0-100) */
	float battery;              /**< Battery Level (0-100) */
	float capacity;				/**< Battery Level (0-100) */
	string context;             /**< Context (outdoor, semi-outdoor, indoor) */
	DatatbdelM dtbd;            /**< Amount of data to be delivered */
	DatasentM dts;              /**< Amount of data already sent */
	Samples smp;                /**<  List of samples */
	int stop;                   /**< flag for DDF */
	vector<pair<float, float> > route;  /**< define route of user*/
	vector<bool> generate;		/**< for each location of route generate or not */
	bool gen;					/**< flag if generate or not */
	bool flagstart;
	int numgeneration;
	int duration;
	int minwalk;
	int goai;
    vector<int> timeroute;
    int idgo;
    int members;
Smartphone(int A,float sB, float B,float Ca, string (C), DatatbdelM (D), DatasentM (E), Samples(F),int s,vector<pair<float, float> > r,vector<bool> G,bool g,bool fl,int num,int dur,int mn,int go,vector<int> tim, int ig,int mem): id_user(A),startBattery(sB), battery(B),capacity(Ca), context(C), dtbd(D), dts(E), smp(F) , stop(s), route(r),generate(G),gen(g),flagstart(fl),numgeneration(num),duration(dur),minwalk(mn),goai(go),timeroute(tim),idgo(ig),members(mem){}

Smartphone(){}


};

typedef std::map<int,Smartphone> Smartphones;

/**
 *  It sets the details of the smartphone for each user
 */
Smartphones setSmartphones(int num_users);

#endif /* SMARTPHONES_H_ */












