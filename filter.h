#ifndef madgwickFilter_h
#define madgwickFilter_h


#include "util.h"
#define gyroMeanError PIf * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s) *from paper*
#define gyroMeanDrift PIf * (0.2f / 180.0f)
#define betaEventual 0.866025f * gyroMeanError    //*from paper*
#define zeta 0.866025f * gyroMeanDrift
#define betaStart 2.5;


extern Quaternion q_est;
extern Quaternion omega_est;



extern void filter(
    Vec3 g,
    Vec3 a,
    Vec3 m,
    Vec3 n1,
    Vec3 n2,
    Vec3 n3,
    Vec3 I,
    float lat, float lon, float t, int mode, float deltaT
);

extern float test(float x);
extern void init_filter();
extern void normal();

#endif /* madgwickFilter_h */