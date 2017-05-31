#ifndef WAVE_VISION_DATASET_HPP
#define WAVE_VISION_DATASET_HPP

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"
#include "wave/kinematics/two_wheel.hpp"

namespace wave {

class TestCamera {
 public:
    int image_width;
    int image_height;
    Mat3 K;
    double hz;

    double dt;
    int frame;

    TestCamera()
        : image_width{-1}, image_height{-1}, K{}, hz{-1}, dt{0}, frame{-1} {}

    bool update(double dt);
    int checkFeatures(double dt,
                      const MatX &features,
                      const Vec3 &rpy,
                      const Vec3 &t,
                      std::vector<std::pair<Vec2, Vec3>> &observed);
};

class TestDataset {
 public:
    TestCamera camera;
    int nb_features;
    Vec2 feature_x_bounds;
    Vec2 feature_y_bounds;
    Vec2 feature_z_bounds;

    TestDataset()
        : camera{},
          nb_features{-1},
          feature_x_bounds{},
          feature_y_bounds{},
          feature_z_bounds{} {}
    explicit TestDataset(const std::string &config_file);

    void generateRandom3DFeatures(MatX &features);
    void record3DFeatures(const std::string &output_path, const MatX &features);
    void recordObservedFeatures(double time,
                                const Vec3 &x,
                                const std::string &output_path,
                                std::vector<std::pair<Vec2, Vec3>> &observed);
    void generateTestData(const std::string &output_path);
};

}  // wave namespace
#endif
