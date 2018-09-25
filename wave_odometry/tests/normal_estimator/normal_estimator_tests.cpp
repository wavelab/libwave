#include <sys/stat.h>
#include "wave/wave_test.hpp"
#include "wave/odometry/geometry/SRI_normals.hpp"

namespace {

void getInput(Eigen::Tensor<float, 2>* retval) {
    std::fstream cloud_file;
    std::string filename = "/home/ben/rosbags/wat_27/lidar/0000000000.bin";
    cloud_file.open(filename, std::ios::in | std::ios::binary);
    cloud_file.ignore(std::numeric_limits<std::streamsize>::max());
    auto length = cloud_file.gcount();
    cloud_file.clear();
    cloud_file.seekg(0, std::ios_base::beg);

    if (!cloud_file.good()) {
        throw std::runtime_error("Cannot find pointcloud file");
    }

    constexpr int pt_size = sizeof(float) * 7 + sizeof(uint16_t) + sizeof(uint32_t);
    long cloud_size = (length / pt_size) + 1;

    retval->resize(3, cloud_size);
    long counter = 0;
    while (cloud_file.good() && !cloud_file.eof()) {
        if (counter == cloud_size) {
            throw std::runtime_error("Filesize isn't working right");
        }
        float x, y, z;
        float rng, el, az;
        uint16_t ring;
        int32_t nanosec_offset;
        float intensity;

        cloud_file.read((char *) &x, sizeof(float));
        cloud_file.read((char *) &y, sizeof(float));
        cloud_file.read((char *) &z, sizeof(float));
        cloud_file.read((char *) &(intensity), sizeof(float));
        cloud_file.read((char *) &rng, sizeof(float));
        cloud_file.read((char *) &az, sizeof(float));
        cloud_file.read((char *) &el, sizeof(float));
        cloud_file.read((char *) &(ring), sizeof(uint16_t));
        cloud_file.read((char *) &nanosec_offset, sizeof(int32_t));
        
        if (std::isnan(x)) {
            continue;
        }

        (*retval)(0,counter) = az;
        (*retval)(1,counter) = el;
        (*retval)(2,counter) = rng;
        ++counter;
    }
    if (counter != cloud_size) {
        throw std::runtime_error("Tensor created was too big");
    }
}

}

namespace wave {

TEST(normal_estimation_tests, constructor) {
    NormalEstimatorParams params;
    NormalEstimator estimator(params);
}

TEST(normal_estimation_tests, setupMasks) {
    NormalEstimatorParams params;
    NormalEstimator estimator(params);

    Eigen::Tensor<float, 2> input_scan;
    getInput(&input_scan);

    estimator.setInput(input_scan, 32);
    const auto& normal_mask = estimator.getNormalMask();
    const auto& interpolate_mask = estimator.getInterpolationMask();

    std::ofstream normal_mat("/home/ben/normal_mask.txt");
    std::ofstream int_mat("/home/ben/int_mask.txt");
    for (int r_id = 0; r_id < normal_mask.dimension(0); ++r_id) {
        for(int c_id = 0; c_id < normal_mask.dimension(1); ++c_id) {
            normal_mat << (int) (normal_mask(r_id, c_id)) << " ";
            int_mat << (int) (interpolate_mask(r_id, c_id)) << " ";
        }
        normal_mat << "\n";
        int_mat << "\n";
    }
}

TEST(normal_estimation_tests, interpolate) {
    NormalEstimatorParams params;
    NormalEstimator estimator(params);

    Eigen::Tensor<float, 2> input_scan;
    getInput(&input_scan);
    estimator.setInput(input_scan, 32);
    estimator.interpolate();

    const auto& interpolated_image = estimator.getInterpolatedImage();
    std::ofstream image_mat("/home/ben/image.txt");
    for (int r_id = 0; r_id < interpolated_image.dimension(0); ++r_id) {
        for(int c_id = 0; c_id < interpolated_image.dimension(1); ++c_id) {
            image_mat << (interpolated_image(r_id, c_id)) << " ";
        }
        image_mat << "\n";
    }
}

}
