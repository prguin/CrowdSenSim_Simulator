/*
 * Utilities.cc

 *
 *  Created on: 30 giu 2016
 *  Author: Giuseppe Cacciatore
 */

#include "../Headers/Utilities.h"

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
#include <iterator>
#include <math.h>


// Inputs (and outputs) are in Degrees, Meters
void destVincenty(double lat1, double lon1, double bearing, double dist,double *lat2out, double *lon2out){
  /* local variable definitions */

  // WGS-84 ellipsiod
  double a=6378137.0, b=6356752.3142, f=1/298.257223563;
  double alpha1,sinAlpha, sinAlpha1, cosAlpha1, cosSqAlpha;
  double sigma, sigma1, cos2SigmaM, sinSigma, cosSigma, deltaSigma, sigmaP;
  double tanU1, cosU1, sinU1, uSq;
  double A, B, C, L, lambda;
  double tmp, lat2;
  //double revAz;   /* unused but retained for alg completeness */

  /* code body */

  alpha1 = bearing*DD2R;
  sinAlpha1 = sin(alpha1);
  cosAlpha1 = cos(alpha1);

  tanU1 = (1-f) * tan(lat1*DD2R);
  cosU1 = 1 / sqrt((1 + tanU1*tanU1));
  sinU1 = tanU1*cosU1;
  sigma1 = atan2(tanU1, cosAlpha1);
  sinAlpha = cosU1 * sinAlpha1;
  cosSqAlpha = 1 - sinAlpha*sinAlpha;
  uSq = cosSqAlpha * (a*a - b*b) / (b*b);
  A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
  B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));

  sigma = dist / (b*A);
  sigmaP = 2*DPI;
  while (fabs(sigma-sigmaP) > 1e-12) {
    cos2SigmaM = cos(2*sigma1 + sigma);
    sinSigma = sin(sigma);
    cosSigma = cos(sigma);
    deltaSigma = B*sinSigma*(cos2SigmaM+B/4*(cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)-B/6*cos2SigmaM*(-3+4*sinSigma*sinSigma)*(-3+4*cos2SigmaM*cos2SigmaM)));
    sigmaP = sigma;
    sigma = dist / (b*A) + deltaSigma;
  }

  tmp = sinU1*sinSigma - cosU1*cosSigma*cosAlpha1;
  lat2 = atan2(sinU1*cosSigma + cosU1*sinSigma*cosAlpha1,
      (1-f)*sqrt(sinAlpha*sinAlpha + tmp*tmp));
  lambda = atan2(sinSigma*sinAlpha1,
                 cosU1*cosSigma - sinU1*sinSigma*cosAlpha1);
  C = f/16*cosSqAlpha*(4+f*(4-3*cosSqAlpha));
  L = lambda - (1-C)*f*sinAlpha*(sigma+C*sinSigma*(cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)));

  // final bearing
  // revAz = atan2(sinAlpha, -tmp);

  // algorithm convention uses Deg outputs
  *lat2out = lat2*DR2D;
  *lon2out = lon1+(L*DR2D);
}

double fRand(double fMin, double fMax){
/*
 * It returns a random double number between a maximum and minimum value.
 */

	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

long rnd32(long seed){
    long times, rest, prod1, prod2;

    times = seed / LASTXN;
    rest  = seed - times * LASTXN;
    prod1 = times * UPTOMOD;
    prod2 = rest * MYA;
    seed  = prod1 + prod2;
    if (seed < 0) seed = seed + MODULE;
    return (seed);
}

double uniform(double a, double b, long *seed){
    double u;
    *seed = rnd32 (*seed);
    u = (*seed) * RATIO;
    u = a + u * (b-a);
    return (u);
}

#define pi 3.14159265358979323846

#define R 6371
#define TO_RAD (3.1415926536 / 180)

double havdist(double th1, double ph1, double th2, double ph2){
/*
 * It computes the Haversine distance between two points in the map.
 */

	double dx, dy, dz;
	ph1 -= ph2;
	ph1 *= TO_RAD, th1 *= TO_RAD, th2 *= TO_RAD;

	dz = sin(th1) - sin(th2);
	dx = cos(ph1) * cos(th1) - cos(th2);
	dy = sin(ph1) * cos(th1);
	return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R;
}
std::string PrintByteUnit(long double bytes){
/*
 * It is useful for printing the exactly bytes unit for an input bytes value.
 */

string output_bytes;
std::stringstream tmp_outbuff;
long double temp_bytes;

if(bytes >=1099511627776){
	 temp_bytes= bytes/1099511627776;
	 tmp_outbuff<<fixed<<setprecision(2)<<temp_bytes<<" TiB";
	 output_bytes  = tmp_outbuff.str();
}else if (bytes >= 1073741824){
	temp_bytes= bytes/1073741824;
	tmp_outbuff<<fixed<<setprecision(2)<<temp_bytes<<" GiB";
	output_bytes  = tmp_outbuff.str();
}else if (bytes >= 1048576){
	temp_bytes= bytes/1048576;
	tmp_outbuff<<fixed<<setprecision(2)<<temp_bytes<<" MiB";
	output_bytes  = tmp_outbuff.str();
}else if (bytes >= 1024){
	temp_bytes= bytes/1024;
	tmp_outbuff<<fixed<<setprecision(2)<<temp_bytes<<" KiB";
	output_bytes  = tmp_outbuff.str();
}else if (bytes > 1){
	tmp_outbuff<<fixed<<setprecision(2)<<bytes<<" B";
	output_bytes  = tmp_outbuff.str();
}

return output_bytes;
}

std::string PrintByteUnitInMiB(long double bytes){
/*
 * It computes the bytes in MiB for the bytes input.
 */

string output_bytes;
std::stringstream tmp_outbuff;
long double temp_bytes;

temp_bytes= bytes/1048576;
	tmp_outbuff<<fixed<<setprecision(2)<<temp_bytes;
	output_bytes  = tmp_outbuff.str();
return output_bytes;
}

