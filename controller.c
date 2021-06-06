#include "controller.h"




void controller(Vec3* T_c, Quaternion q_est, Quaternion q_ref, Quaternion omega_est) {
    Quaternion q_err = quat_mult(q_ref, quat_conjugate(q_est));
    Quaternion q_err_dot = quat_mult(q_err, omega_est);
    quat_scalar(&q_est_dot, 0.5f);

    T_c->x = -q_err.q2*K_P - q_err_dot.q2*K_D;
    T_c->y = -q_err.q3*K_P - q_err_dot.q3*K_D;
    T_c->z = -q_err.q4*K_P - q_err_dot.q4*K_D;
}