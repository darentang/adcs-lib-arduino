#include "adcs.h"


ADCS::ADCS() {
    init_filter();
}

void ADCS::set_normal() {
    normal();
}

void ADCS::update(    
            Vec3 g,
            Vec3 a,
            Vec3 m,
            Vec3 n1,
            Vec3 n2,
            Vec3 n3,
            Vec3 I,
            float lat, float lon, float t, int mode, float deltaT) 
{
    
    filter(g, a, m, n1, n2, n3, I, lat, lon ,t, mode, deltaT);
    this->q_est_ = q_est;
    this->omega_est_ = omega_est;
}

void ADCS::set_point(Quaternion q_ref) {
    controller(&T_c, q_ref, q_est, omega_est);
}
