#include "slam/estimation/pf.hpp"


namespace slam {

// ParticleFilter::ParticleFilter(void)
// {
//     this->initialized = false;
// }
//
// int ParticleFilter::init(int M, VecX mu)
// {
//     // int nb_states;
//     //
//     // nb_states = mu.size();
//     // this->initialized = true;
//     // this->M = M;
//     // this->mu = mu;
//
//     // this->mu_p = VecX::Zero(nb_states);
//     // this->S_p = MatX::Zero(nb_states, nb_states);
//
//     return 0;
// }
//
// int ParticleFilter::estimate(
//     std::vector<VecX> X_p,
//     std::vector<VecX> hX_p,
//     VecX y
// )
// {
// //     // % sampling
// //     // for m = 1:pf.M
// //     //     Xp = feval(Xp_func, pf.X(m), u, pf.R);
// //     //     hXp = feval(hXp_func, Xp);
// //     //
// //     //     pf.Xp(:, m) = Xp;
// //     //     pf.w(m) = max(1e-8, mvnpdf(y, hXp, pf.Q));
// //     // end
// //
// //     // % importance resampling
// //     // W = cumsum(pf.w);
// //     // for m = 1:pf.M
// //     //     seed = W(end) * rand(1);
// //     //     pf.X(m) = pf.Xp(find(W > seed, 1));
// //     // end
// //
// //     // % record mean particle
// //     // pf.mu = mean(pf.X);
// //     // pf.S = var(pf.X);
// //
// //     for (int i; i < this->M; i++) {
// //
// //     }
//
//     VecX x_p;
//     VecX hx_p;
//
//     for (int i = 0; i < this->M; i++) {
//         x_p = X_p[i];
//         hx_p = hX_p[i];
//
//     }
//
//
//     return 0;
// }

}  // end of slam namespace
