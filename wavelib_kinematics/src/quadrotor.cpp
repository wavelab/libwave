#include "slam/kinematics/quadrotor.hpp"


namespace slam {

QuadrotorModel::QuadrotorModel(void)
{
    this->initialized = false;
}

MatX QuadrotorModel::GFunc(VecX x, VecX u, float dt)
{
    MatX G(12, 12);
    float ph, th, ps, p, q, r, px, py, pz, vx, vy, vz;  // states
    float Ix, Iy, Iz;
    float ktau, kt;
    float tauf, taup, tauq, taur;
    float m, g;

    // setup
    UNUSED(u);

    // states
    ph = x(0);
    th = x(1);
    ps = x(2);
    p = x(3);
    q = x(4);
    r = x(5);
    px = x(6);
    py = x(7);
    pz = x(8);
    vx = x(9);
    vy = x(10);
    vz = x(11);

    // inertia
    Ix = this->Ix;
    Iy = this->Iy;
    Iz = this->Iz;

    // constants
    ktau = this->ktau;
    kt = this->kt;

    // torques
    tauf = this->tauf;
    taup = this->taup;
    tauq = this->tauq;
    taur = this->taur;

    // misc
    m = this->m;
    g = this->g;

    // un-used
    UNUSED(px);
    UNUSED(py);
    UNUSED(pz);
    UNUSED(vx);
    UNUSED(vy);
    UNUSED(vz);
    UNUSED(taup);
    UNUSED(tauq);
    UNUSED(taur);
    UNUSED(g);

    // row 1
    G(0, 0) =  dt*( tan(th)*cos(ph)*q-tan(th)*r*sin(ph))+1.0;
    G(0, 1) = dt*( q*( pow(tan(th),2.0)+1.0)*sin(ph)+cos(ph)*( pow(tan(th),2.0)+1.0)*r);
    G(0, 2) = 0.0;
    G(0, 3) = dt;
    G(0, 4) = dt*tan(th)*sin(ph);
    G(0, 5) = dt*tan(th)*cos(ph);
    G(0, 6) = 0.0;
    G(0, 7) = 0.0;
    G(0, 8) = 0.0;
    G(0, 9) = 0.0;
    G(0, 10) = 0.0;
    G(0, 11) = 0.0;

    // row 2
    G(1, 0) = -dt*( sin(ph)+cos(ph)*r);
    G(1, 1) = 1.0;
    G(1, 2) = 0.0;
    G(1, 3) = 0.0;
    G(1, 4) = dt;
    G(1, 5) = -dt*sin(ph);
    G(1, 6) = 0.0;
    G(1, 7) = 0.0;
    G(1, 8) = 0.0;
    G(1, 9) = 0.0;
    G(1, 10) = 0.0;
    G(1, 11) = 0.0;

    // row 3
    G(2, 0) = -dt*( r*sin(ph)-cos(ph)*q)/cos(th);
    G(2, 1) = ( q*sin(ph)+cos(ph)*r)*dt*sin(th)/pow(cos(th),2.0);
    G(2, 2) = 1.0;
    G(2, 3) = 0.0;
    G(2, 4) = dt/cos(th)*sin(ph);
    G(2, 5) = dt*cos(ph)/cos(th);
    G(2, 6) = 0.0;
    G(2, 7) = 0.0;
    G(2, 8) = 0.0;
    G(2, 9) = 0.0;
    G(2, 10) = 0.0;
    G(2, 11) = 0.0;

    // row 4
    G(3, 0) = 0.0;
    G(3, 1) = 0.0;
    G(3, 2) = 0.0;
    G(3, 3) = -dt*ktau/Ix+1.0;
    G(3, 4) = -dt*r/Ix*( Iz-Iy);
    G(3, 5) = -dt*q/Ix*( Iz-Iy);
    G(3, 6) = 0.0;
    G(3, 7) = 0.0;
    G(3, 8) = 0.0;
    G(3, 9) = 0.0;
    G(3, 10) = 0.0;
    G(3, 11) = 0.0;

    // row 5
    G(4, 0) = 0.0;
    G(4, 1) = 0.0;
    G(4, 2) = 0.0;
    G(4, 3) = dt*( Iz-Ix)*r/Iy;
    G(4, 4) = -dt*ktau/Iy+1.0;
    G(4, 5) = dt*( Iz-Ix)*p/Iy;
    G(4, 6) = 0.0;
    G(4, 7) = 0.0;
    G(4, 8) = 0.0;
    G(4, 9) = 0.0;
    G(4, 10) = 0.0;
    G(4, 11) = 0.0;

    // row 6
    G(5, 0) = 0.0;
    G(5, 1) = 0.0;
    G(5, 2) = 0.0;
    G(5, 3) = dt*q/Iz*( Ix-Iy);
    G(5, 4) = dt/Iz*( Ix-Iy)*p;
    G(5, 5) = -dt/Iz*ktau+1.0;
    G(5, 6) = 0.0;
    G(5, 7) = 0.0;
    G(5, 8) = 0.0;
    G(5, 9) = 0.0;
    G(5, 10) = 0.0;
    G(5, 11) = 0.0;

    // row 7
    G(6, 0) = 0.0;
    G(6, 1) = 0.0;
    G(6, 2) = 0.0;
    G(6, 3) = 0.0;
    G(6, 4) = 0.0;
    G(6, 5) = 0.0;
    G(6, 6) = 1.0;
    G(6, 7) = 0.0;
    G(6, 8) = 0.0;
    G(6, 9) = dt;
    G(6, 10) = 0.0;
    G(6, 11) = 0.0;

    // row 8
    G(7, 0) = 0.0;
    G(7, 1) = 0.0;
    G(7, 2) = 0.0;
    G(7, 3) = 0.0;
    G(7, 4) = 0.0;
    G(7, 5) = 0.0;
    G(7, 6) = 0.0;
    G(7, 7) = 1.0;
    G(7, 8) = 0.0;
    G(7, 9) = 0.0;
    G(7, 10) = dt;
    G(7, 11) = 0.0;

    // row 9
    G(8, 0) = 0.0;
    G(8, 1) = 0.0;
    G(8, 2) = 0.0;
    G(8, 3) = 0.0;
    G(8, 4) = 0.0;
    G(8, 5) = 0.0;
    G(8, 6) = 0.0;
    G(8, 7) = 0.0;
    G(8, 8) = 1.0;
    G(8, 9) = 0.0;
    G(8, 10) = 0.0;
    G(8, 11) = dt;

    // row 10
    G(9, 0) = dt*( cos(ph)*sin(ps)-cos(ps)*sin(th)*sin(ph))*tauf/m;
    G(9, 1) = dt*cos(ph)*tauf*cos(ps)/m*cos(th);
    G(9, 2) = -dt*tauf*( cos(ph)*sin(th)*sin(ps)-cos(ps)*sin(ph))/m;
    G(9, 3) = 0.0;
    G(9, 4) = 0.0;
    G(9, 5) = 0.0;
    G(9, 6) = 0.0;
    G(9, 7) = 0.0;
    G(9, 8) = 0.0;
    G(9, 9) = -dt/m*kt+1.0;
    G(9, 10) = 0.0;
    G(9, 11) = 0.0;

    // row 11
    G(10, 0) = -dt*( sin(th)*sin(ps)*sin(ph)+cos(ph)*cos(ps))*tauf/m;
    G(10, 1) = dt*cos(ph)*tauf/m*cos(th)*sin(ps);
    G(10, 2) = dt*tauf/m*( cos(ph)*cos(ps)*sin(th)+sin(ps)*sin(ph));
    G(10, 3) = 0.0;
    G(10, 4) = 0.0;
    G(10, 5) = 0.0;
    G(10, 6) = 0.0;
    G(10, 7) = 0.0;
    G(10, 8) = 0.0;
    G(10, 9) = 1.0;
    G(10, 10) = -dt/m*kt;
    G(10, 11) = 0.0;

    // row 12
    G(11, 0) = -dt*tauf/m*cos(th)*sin(ph);
    G(11, 1) = -dt*cos(ph)*tauf*sin(th)/m;
    G(11, 2) = 0.0;
    G(11, 3) = 0.0;
    G(11, 4) = 0.0;
    G(11, 5) = 0.0;
    G(11, 6) = 0.0;
    G(11, 7) = 0.0;
    G(11, 8) = 0.0;
    G(11, 9) = 1.0;
    G(11, 10) = 0.0;
    G(11, 11) = -dt/m*kt;

    return G;
}

} // end of slam namespace
