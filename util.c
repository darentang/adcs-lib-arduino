#include "util.h"
#include <math.h>
#include <stdio.h>

void ECEFtoECI(Vec3 inVec, Vec3 * outVec, float time) {
    float JD2000 = UTCtoJD2000(time);
    float theta = 4.89496121274f + 6.30038809899f * JD2000;
    outVec->x = cosf(theta) * inVec.x - sinf(theta) * inVec.y;
    outVec->y = sinf(theta) * inVec.x + cosf(theta) * inVec.y;
    outVec->z = inVec.z;
}

void ECItoECEF(Vec3 inVec, Vec3 *outVec, float time) {
    float JD2000 = UTCtoJD2000(time);
    float theta = 4.89496121274f + 6.30038809899f * JD2000;
    outVec->x = cosf(theta) * inVec.x + sinf(theta) * inVec.y;
    outVec->y = -sinf(theta) * inVec.x + cosf(theta) * inVec.y;
    outVec->z = inVec.z;
}

void ECEFtoNED(Vec3 inVec, Vec3 * outVec, float lat, float lon) {
    float tau = lon * PIf / 180.0f;
    float mu = lat * PIf / 180.0f;
    outVec->x = -inVec.x * sinf(mu) * cosf(tau) - inVec.y * sinf(mu) * sinf(tau) + inVec.z * cosf(mu);
    outVec->y = -inVec.x * sinf(tau) + inVec.y * cosf(tau);
    outVec->z = -inVec.x * cosf(mu) * cosf(tau) - inVec.y * cosf(mu) * sinf(tau) - inVec.z * sinf(mu);
}

void NEDtoECEF(Vec3 inVec, Vec3 * outVec, float lat, float lon) {
    float tau = lon * PIf / 180.0f;
    float mu = lat * PIf / 180.0f;
    outVec->x = -inVec.x * sinf(mu) * cosf(tau) - inVec.y * sinf(tau) - inVec.z * cosf(mu) * cosf(tau);
    outVec->y = -inVec.x * sinf(mu) * sinf(tau) + inVec.y * cosf(tau) - inVec.z * cosf(mu) * sinf(tau);
    outVec->z = inVec.x * cosf(mu) + -inVec.z * sinf(mu);
}

void printVec3(Vec3 vec) {
    printf("[%f, %f, %f]\n", vec.x, vec.y, vec.z);
}

void normVec3(Vec3* v) {
    float n = sqrtf(powf(v->x, 2) + powf(v->y, 2) + powf(v->z, 2));
    if (n == 0.0f) {
        return;
    }
    v->x /= n;
    v->y /= n;
    v->z /= n;
}
