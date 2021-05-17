#ifndef magVector_h
#define magVector_h

// #include "wmm.h"
#include "util.h"

extern Vec3 bVec_ECEF;
extern Vec3 bVec_ECI;
extern Vec3 bVec_NED;

void magVector(float lat, float lon, float t, Vec3 h);
float magDeclination(float lat, float lon);

#endif