#ifndef controller_h
#define controller_h

#include "util.h"

const float K_P = -0.4f/4000.0f;
const float K_D = 2.0f/300.0f;


extern void controller(Vec3* T_c, Quaternion q_est, Quaternion q_ref, Quaternion omega_est);

#endif