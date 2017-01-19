#ifndef __wave_SYMMATH_MODELS_HPP__
#define __wave_SYMMATH_MODELS_HPP__

#include <iostream>
#include <fstream>

#include <ginac/ginac.h>

namespace wave {

void quadrotor_jacobian(
    std::vector<GiNaC::ex> &model,
    std::vector<GiNaC::symbol> &states
);

void bundle_adjustment_jacobian(std::string file_path);

} // end of wave namespace
#endif
