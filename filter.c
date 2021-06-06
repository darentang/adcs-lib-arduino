// Implementation of the Madgwick filter with sunsensors
// Author: Daren Tang, 2021

#include "filter.h"
#include "sunVector.h"
#include "magVector.h"
#include <math.h>

Quaternion q_est = {1, 0, 0, 0};
Quaternion omega_bias = {0, 0, 0, 0};
Quaternion omega_est = {0, 0, 0, 0};
Quaternion q_est_dot = {0, 0, 0, 0};

float beta = betaStart;

Quaternion quat_mult (Quaternion L, Quaternion R){    
    Quaternion product;
    product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
    product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
    product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
    product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);
    
    return product;
}


/**
 * Calculate vector rotation
 * @param q quaternion of orientation
 * @param d vector to rotate
 * @param f pointer of vector to store results
 **/
void quat_transform(Quaternion q, Vec3 d, Vec3 * f) {
    f->x = 2 * d.x * (0.5 - q.q3*q.q3 - q.q4*q.q4)
        + 2 * d.y * (q.q1*q.q4 + q.q2*q.q3)
        + 2 * d.z * (q.q2*q.q4 - q.q1*q.q3);
    f->y = 2 * d.x * (q.q2*q.q3 - q.q1*q.q4)
        + 2 * d.y * (0.5 - q.q2*q.q2 - q.q4*q.q4)
        + 2 * d.z * (q.q1*q.q2 + q.q3*q.q4);
    f->z = 2 * d.x * (q.q1*q.q3 + q.q2*q.q4)
        + 2 * d.y * (q.q3*q.q4 - q.q1*q.q2) 
        + 2 * d.z * (0.5 - q.q2*q.q2 - q.q3*q.q3);
}

/**
 * Calculate the gradient in optimisation step
 * @param q attitude
 * @param d earth frame ref vector
 * @param s sensor frame measurement
 * @param q_est_dot pointer to quaternion to store result
 **/
void gradient(Quaternion q, Vec3 d, Vec3 s, Quaternion *q_est_dot) {    
    Vec3 f;
    struct jacobian J;
    float q1 = q.q1;
    float q2 = q.q2;
    float q3 = q.q3;
    float q4 = q.q4;

    quat_transform(q, d, &f);
    f.x -= s.x;
    f.y -= s.y;
    f.z -= s.z;


    J.a11 = 2*d.y*q4 - 2*d.z*q3; 
    J.a12 = 2*d.y*q3 + 2*d.z*q4; 
    J.a13 = -4*d.x*q3 + 2*d.y*q2 - 2*d.z*q1;
    J.a14 = -4*d.x*q4 + 2*d.y*q1 - 2*d.z*q2;
    
    J.a21 = -2*d.x*q4 + 2*d.z*q2; 
    J.a22 = 2*d.x*q3 - 4*d.y*q2 + 2*d.z*q1; 
    J.a23 = 2*d.x*q2 + 2*d.z*q4;
    J.a24 = -2*d.x*q1 - 4*d.y*q4;

    J.a31 = 2*d.z*q3 - 2*d.y*q2;
    J.a32 = 2*d.y*q4 - 2*d.y*q1 - 4*d.z*q2;
    J.a33 = 2*d.x*q1 + 2*d.y*q4 - 4*d.z*q3;
    J.a34 = 2*d.x*q2 + 2*d.y*q3;

    jacobianMult(f, J, q_est_dot);
};

Vec3 quatToVec(Quaternion q) {
    Vec3 x = {q.q2, q.q3, q.q4};
    return x;
}

Quaternion vecToQuat(Vec3 v) {
    Quaternion x = {0, v.x, v.y, v.z};
    return x;
}

void reset() {
    q_est.q1 = 1;
    q_est.q2 = 0;
    q_est.q3 = 0;
    q_est.q4 = 0;
}

void init_filter() {
    // wmm_init();
    beta = betaStart;
}

void normal() {
    beta = betaEventual;
}

/**
 * estimate in NED frame
 * @param g gyro sensor data
 * @param a accelorometer data
 * @param m compass data
 * @param n1, n2, n3 normal vector for sensor 1, 2, 3
 * @param I intensity at sensor n1, n2, n3
 * @param lat latitude
 * @param lon longitude
 * @param t UNIX time
 * @param mode mode selection
 **/
void filter(Vec3 g, Vec3 a, Vec3 m, Vec3 n1, Vec3 n2, Vec3 n3, Vec3 I,
            float lat, float lon, float t, int mode, float deltaT
    ) {

    Quaternion q_est_prev = q_est;
    
    Quaternion q_est_dot_m;
    Quaternion q_est_dot_a;
    Quaternion q_est_dot_s;

    q_est_dot.q1 = 0;
    q_est_dot.q2 = 0;
    q_est_dot.q3 = 0;
    q_est_dot.q4 = 0;

    Quaternion g_q = {0, g.x, g.y, g.z};
    // normalise vectors
    normVec3(&a);
    normVec3(&m);

    Vec3 h;
    quat_transform(quat_conjugate(q_est_prev), m, &h);
    magVector(lat, lon, t, h);
    gradient(q_est, bVec_NED, m, &q_est_dot_m);
    quat_add(&q_est_dot, q_est_dot, q_est_dot_m);


    
    // using accelorometer
    if (mode == 0 || mode == 2) {
        const Vec3 g_ref = {0, 0, 1};
        gradient(q_est, g_ref, a, &q_est_dot_a);
        quat_add(&q_est_dot, q_est_dot, q_est_dot_a);
    }

    // using sun sensor
    if (mode == 1 || mode == 2) {
        sunSensorEstimate(n1, n2, n3, I);
        sunVector(lat, lon, t);
        gradient(q_est,  sVec_NED, sVec_SCB, &q_est_dot_s);
        
        quat_add(&q_est_dot, q_est_dot, q_est_dot_s);
    }

    quat_Normalization(&q_est_dot);

    // equation (47)
    Quaternion omega_epsilon = quat_mult(quat_conjugate(q_est_prev), q_est_dot);
    quat_scalar(&omega_epsilon, 2.0f);
    
    // equation (48)
    omega_bias.q1 += omega_epsilon.q1*deltaT;
    omega_bias.q2 += omega_epsilon.q2*deltaT;
    omega_bias.q3 += omega_epsilon.q3*deltaT;
    omega_bias.q4 += omega_epsilon.q4*deltaT;

    // equation (49)
    omega_est.q1 = g_q.q1 - zeta*omega_bias.q1;
    omega_est.q2 = g_q.q2 - zeta*omega_bias.q2;
    omega_est.q3 = g_q.q3 - zeta*omega_bias.q3;
    omega_est.q4 = g_q.q4 - zeta*omega_bias.q4;

    // equation (12)
    Quaternion q_omega_dot = quat_mult(q_est_prev, omega_est);
    quat_scalar(&q_omega_dot, 0.5f);
    
    // equation (43)
    q_est_dot.q1 = q_omega_dot.q1 - beta * q_est_dot.q1;
    q_est_dot.q2 = q_omega_dot.q2 - beta * q_est_dot.q2;
    q_est_dot.q3 = q_omega_dot.q3 - beta * q_est_dot.q3;
    q_est_dot.q4 = q_omega_dot.q4 - beta * q_est_dot.q4;

    
    // equation (42)
    q_est.q1 = q_est_prev.q1 + q_est_dot.q1 * deltaT;
    q_est.q2 = q_est_prev.q2 + q_est_dot.q2 * deltaT;
    q_est.q3 = q_est_prev.q3 + q_est_dot.q3 * deltaT;
    q_est.q4 = q_est_prev.q4 + q_est_dot.q4 * deltaT;

    // calculate omega estimate
    omega_est = quat_mult(quat_conjugate(q_est), q_est_dot);
    quat_scalar(&omega_est, 2.0f);

    quat_Normalization(&q_est);
}


float quat_Norm (Quaternion q){
    return sqrt(q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3 +q.q4*q.q4);
}

float test(float x) {
    return x * 2.0f;
}
