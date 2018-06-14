/**
 * Defines a struct used for point information in implicit residuals
 */

#ifndef WAVE_POINT_META_DATA_HPP
#define WAVE_POINT_META_DATA_HPP

#include <stdint-gcc.h>

namespace wave {

struct PtMetaData {
    uint32_t prev, next;
    /// Placeholders until I figure out what is should be
    double *t1, *t2, *t3;
};

}

#endif //WAVE_POINT_META_DATA_HPP
