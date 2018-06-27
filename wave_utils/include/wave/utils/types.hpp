#ifndef WAVE_TYPES_HPP
#define WAVE_TYPES_HPP

#include <Eigen/Eigen>

namespace wave {

template<typename T>
using Vec = std::vector<T>;

template<typename T>
using VecE = std::vector<T, Eigen::aligned_allocator<T>>;

}

#endif //WAVE_TYPES_HPP
