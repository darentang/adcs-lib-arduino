#include "magVector.h"
#include <stdio.h>
#include <math.h>

// magnetic field vector
Vec3 bVec_NED = {0, 0, 0};
Vec3 bVec_ECEF = {0, 0, 0};
Vec3 bVec_ECI = {0, 0, 0};
// float dec = 12.0f;
float dec = 0.0f;

float magDeclination(float lat, float lon) {
    float x;
    // E0000(lat, lon, 2021.0, &x);
    return x;
}

void magVector(float lat, float lon, float t, Vec3 h) {
    // E0000(lat, lon, 2021.0, &dec);
    bVec_NED.x = cosf(dec * PIf / 180.0f) * sqrtf(h.x*h.x + h.y*h.y);
    bVec_NED.y = sinf(dec * PIf / 180.0f) * sqrtf(h.x*h.x + h.y*h.y);
    bVec_NED.z = h.z;
    // bVec_NED.x = cosf(dec * PIf / 180.0f);
    // bVec_NED.y = sinf(dec * PIf / 180.0f);
    // bVec_NED.z = 0.0f;
    NEDtoECEF(bVec_NED, &bVec_ECEF, lat, lon);
    ECEFtoECI(bVec_ECEF, &bVec_ECI, t);
}

