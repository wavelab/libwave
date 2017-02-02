#ifndef __wave_SYMMATH_SYMMATH_HPP__
#define __wave_SYMMATH_SYMMATH_HPP__

#include <iostream>
#include <vector>
#include <fstream>

#include <ginac/ginac.h>


namespace wave {

void output_jacobian(std::string file_path,
                     std::vector<GiNaC::ex> model,
                     std::vector<GiNaC::symbol> states);

}  // end of wave namespace
#endif
