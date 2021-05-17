#ifndef adcs_h
#define adcs_h
extern "C" {
#include "filter.h"
#include "controller.h"
}
class ADCS
{
    public:
        ADCS();
        void update(    
            Vec3 g,
            Vec3 a,
            Vec3 m,
            Vec3 n1,
            Vec3 n2,
            Vec3 n3,
            Vec3 I,
            float lat, float lon, float t, int mode, float deltaT);
        void set_point(Quaternion q_ref);
        void set_normal();
        Quaternion q_est_;
        
        Quaternion omega_est_;
        Vec3 T_c;
};


#endif