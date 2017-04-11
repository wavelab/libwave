#ifndef WAVE_VISION_DATASET_HPP
#define WAVE_VISION_DATASET_HPP

#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"

namespace wave {

class TestCamera {
 public:
    int image_width;
    int image_height;
    Mat3 K;
    double hz;

    double dt;

    TestCamera(void);
    bool update(double dt);
};

class TestDataset {
 public:
    bool configured;

    TestCamera camera;
    int nb_features;
    Vec2 feature_x_bounds;
    Vec2 feature_y_bounds;
    Vec2 feature_z_bounds;

    TestDataset(void);
    int configure(std::string config_file);
    int generateRandom3DFeatures(MatX &features);
    int generateTestData(std::string output_path);
};

}  // end of wave namespace
#endif
