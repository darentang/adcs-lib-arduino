// sunVector.h

#ifndef sunVector_h
#define sunVector_h


#define MSUNEPOCH 357.528f
#define NSUN 0.9856003f

#include "util.h"
#include <time.h>

extern Vec3 sVec_ECI;
extern Vec3 sVec_ECEF;
extern Vec3 sVec_NED;
extern Vec3 sVec_SCB;


extern void sunVector(float lat, float lon, float t);
extern void getSunVector(float lat, float lon, float t, Vec3* out);

void sunSensorEstimate(Vec3 n1, Vec3 n2, Vec3 n3, Vec3 I);


#endif