#ifndef __wavelib_SYMMATH_MODELS_HPP__
#define __wavelib_SYMMATH_MODELS_HPP__

#include <iostream>
#include <fstream>

#include <ginac/ginac.h>

namespace wavelib {

void quadrotor_jacobian(
    std::vector<GiNaC::ex> &model,
    std::vector<GiNaC::symbol> &states
);

void bundle_adjustment_jacobian(std::string file_path);

} // end of wavelib namespace
#endif
