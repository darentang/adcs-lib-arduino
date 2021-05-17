// calculate and return the sun vector at time t (UNIX time)

#include "sunVector.h"
#include <math.h>

// sun unit vector
Vec3 sVec_ECI = {0, 0, 0};
Vec3 sVec_ECEF = {0, 0, 0};
Vec3 sVec_NED = {0, 0, 0};
Vec3 sVec_SCB = {0, 0, 0};
/**
 * Param: t seconds since unix epoch
 * 
 **/
void sunVector(float lat, float lon, float t) {
    float JD2000 = UTCtoJD2000(t);
    float MSun = MSUNEPOCH + NSUN * JD2000;

    float VSun = 280.461f + 0.9856474f * JD2000 + 
        1.915f * sinf(MSun * PIf / 180.0f ) + 0.020f * sinf(2 * MSun * PIf / 180.0f );
    float epsilon = 23.4393f - (46.8093f/3600.0f) * JD2000 / 36525.0f;

    sVec_ECI.x = cosf(VSun * PIf / 180.0f);
    sVec_ECI.y = cosf(epsilon * PIf / 180.0f) * sinf(VSun * PIf / 180.0f);
    sVec_ECI.z = sinf(epsilon * PIf / 180.0f) * sinf(VSun * PIf / 180.0f);
    
    ECItoECEF(sVec_ECI, &sVec_ECEF, t);
    ECEFtoNED(sVec_ECEF, &sVec_NED, lat, lon);
}

void getSunVector(float lat, float lon, float t, Vec3* out) {
    sunVector(lat, lon ,t);
    out->x = sVec_NED.x;
    out->y = sVec_NED.y;
    out->z = sVec_NED.z;
}

/** 
 * Calculate estimate sun direction in body frame
 * 
 * @param n[1,2,3][x, y, z] sensor number and x, y, z normal component
 * @param I1, I2, I3 intensity in sensor 1, 2 and 3
 **/

void sunSensorEstimate(
    Vec3 n1, Vec3 n2, Vec3 n3, Vec3 I
) {
    struct mat3 X;
    X.a11 = n1.x; X.a12 = n1.y; X.a13 = n1.z;
    X.a21 = n2.x; X.a22 = n2.y; X.a23 = n2.z;
    X.a31 = n3.x; X.a32 = n3.y; X.a33 = n3.z;

    Vec3 y = {I.x, I.y, I.z};
    struct mat3 X_inv;
    invMat3(X, &X_inv);
    vecmult3(X, y, &sVec_SCB);
}