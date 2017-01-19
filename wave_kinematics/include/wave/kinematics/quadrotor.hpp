#ifndef __wave_KINEMATICS_QUADROTOR_HPP__
#define __wave_KINEMATICS_QUADROTOR_HPP__

#include "wave/utils/utils.hpp"


namespace wave {

class QuadrotorModel
{
public:
    bool initialized;

    float Ix;
    float Iy;
    float Iz;

    float ktau;
    float kt;

    float tauf;
    float taup;
    float tauq;
    float taur;

    float m;
    float g;

    QuadrotorModel(void);
    VecX gFunc(VecX x, VecX u, float dt);
    MatX GFunc(VecX x, VecX u, float dt);
    VecX hFunc(VecX x);
    MatX HFunc(VecX y);
};

} // end of wave namespace
#endif
