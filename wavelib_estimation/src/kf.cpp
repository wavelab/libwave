#include "slam/estimation/kf.hpp"


namespace slam {

KalmanFilter::KalmanFilter(void)
{
    this->initialized = false;
}

int KalmanFilter::init(VecX mu, MatX R, MatX C, MatX Q)
{
    int nb_states;

    nb_states = mu.size();
    this->initialized = true;
    this->mu = mu;

    this->B = MatX::Zero(nb_states, nb_states);
    this->R = R;

    this->C = C;
    this->Q = Q;

    this->S = MatX::Identity(nb_states, nb_states);
    this->I = MatX::Identity(nb_states, nb_states);
    this->K = MatX::Zero(nb_states, nb_states);

    this->mu_p = VecX::Zero(nb_states);
    this->S_p = MatX::Zero(nb_states, nb_states);

    return 0;
}

int KalmanFilter::estimate(MatX A, VecX y)
{
    // prediction update
    mu_p = A * mu;
    S_p = A * S * A.transpose() + R;

    // measurement update
    K = S_p * C.transpose() * (C * S_p * C.transpose() + Q).inverse();
    mu = mu_p + K * (y - C * mu_p);
    S = (I - K * C) * S_p;

    return 0;
}

}  // end of slam namespace
