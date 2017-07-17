#include "wave/wave_test.hpp"
#include "wave/utils/utils.hpp"
#include "wave/vision/dataset.hpp"
#include "wave/optimization/ceres/ba.hpp"

namespace wave {

const std::string TEST_CONFIG = "tests/data/vo_test.yaml";

static double **build_landmark_matrix(const VOTestDataset &dataset) {
    size_t nb_landmarks = dataset.landmarks.size();
    double **landmarks = (double **) malloc(sizeof(double *) * nb_landmarks);

    MatX data = MatX::Zero(nb_landmarks, 3);
    for (auto const &landmark : dataset.landmarks) {
        const auto &point = landmark.second;
        const auto &landmark_id = landmark.first;
        data.block(landmark_id, 0, 1, 3) = point.transpose();
    }

    for (int i = 0; i < data.rows(); i++) {
        landmarks[i] = (double *) malloc(sizeof(double) * 3);
        Vec3 point = data.block(i, 0, 1, 3).transpose();

        landmarks[i][0] = point(0);
        landmarks[i][1] = point(1);
        landmarks[i][2] = point(2);
    }

    return landmarks;
}

static VecX build_landmark_ids(
  const std::vector<LandmarkObservation> &features_observed) {
    VecX landmark_ids{features_observed.size()};

    for (size_t i = 0; i < features_observed.size(); i++) {
        landmark_ids(i) = features_observed[i].first;
    }

    return landmark_ids;
}

static MatX build_feature_matrix(
  const std::vector<LandmarkObservation> &features_observed) {
    MatX features(features_observed.size(), 2);

    for (size_t i = 0; i < features_observed.size(); i++) {
        features.block(i, 0, 1, 2) = features_observed[i].second.transpose();
    }

    return features;
}

static void free_2darray(double **array, size_t nb_elements) {
    for (size_t i = 0; i < nb_elements; i++) {
        free(array[i]);
    }
    free(array);
}

TEST(BAResidual, constructor) {
    // TEST DEFAULT CONSTRUCTOR
    BAResidual residual1;

    EXPECT_FLOAT_EQ(0.0, residual1.fx);
    EXPECT_FLOAT_EQ(0.0, residual1.fy);
    EXPECT_FLOAT_EQ(0.0, residual1.cx);
    EXPECT_FLOAT_EQ(0.0, residual1.cy);
    EXPECT_FLOAT_EQ(0.0, residual1.x);
    EXPECT_FLOAT_EQ(0.0, residual1.y);

    // TEST CONSTRUCTOR WITH PARAMETERS
    Mat3 K;
    // clang-format off
  K << 1.0, 0.0, 3.0,
       0.0, 2.0, 4.0,
       0.0, 0.0, 1.0;
    // clang-format on
    Vec2 x = Vec2{130, 62};

    BAResidual residual2(K, x);
    EXPECT_FLOAT_EQ(K(0, 0), residual2.fx);
    EXPECT_FLOAT_EQ(K(1, 1), residual2.fy);
    EXPECT_FLOAT_EQ(K(0, 2), residual2.cx);
    EXPECT_FLOAT_EQ(K(1, 2), residual2.cy);
    EXPECT_FLOAT_EQ(x(0), residual2.x);
    EXPECT_FLOAT_EQ(x(1), residual2.y);
}

TEST(BAResidual, testResidual) {
    // create vo dataset
    VOTestDatasetGenerator generator;
    generator.configure(TEST_CONFIG);
    auto dataset = generator.generate();
    double **landmarks = build_landmark_matrix(dataset);

    // translate
    const auto &t = dataset.states[0].robot_G_p_B_G;
    Vec3 t_edn = t;
    //    nwu2edn(t, t_edn);
    double *t_vec = (double *) malloc(sizeof(double *) * 3);
    t_vec[0] = t_edn(0);
    t_vec[1] = t_edn(1);
    t_vec[2] = t_edn(2);

    // orientation of robot Body in Global frame
    const auto &q_GB = dataset.states[0].robot_q_GB;

    // rotate the body frame by this to get the camera frame
    // or, use this to transform points from camera frame to body frame
    Quaternion q_BC{Eigen::AngleAxisd(-M_PI_2, Vec3::UnitZ()) *
                    Eigen::AngleAxisd(0, Vec3::UnitY()) *
                    Eigen::AngleAxisd(-M_PI_2, Vec3::UnitX())};

    Quaternion q_GC = q_GB * q_BC;

    double *q_vec = (double *) malloc(sizeof(double *) * 4);
    q_vec[0] = q_GC.w();
    q_vec[1] = q_GC.x();
    q_vec[2] = q_GC.y();
    q_vec[3] = q_GC.z();

    double e[2] = {0.0, 0.0};
    const auto &feature = dataset.states[0].features_observed[0].second;
    const auto &landmark_id = dataset.states[0].features_observed[0].first;

    // test and assert
    BAResidual residual{dataset.camera_K, feature};
    residual(q_vec, t_vec, landmarks[landmark_id], e);
    EXPECT_NEAR(0.0, e[0], 0.0001);
    EXPECT_NEAR(0.0, e[1], 0.0001);

    // clean up
    free_2darray(landmarks, dataset.landmarks.size());
    free(t_vec);
    free(q_vec);
}

// TEST(BundleAdjustment, addCamera) {
//   // setup
//   Mat3 K = Mat3::Identity();
//   MatX features = MatX::Random(10, 2);
//   VecX landmark_ids = VecX::Random(10);
//   Vec3 cam_t{0.0, 0.0, 0.0};
//   Quaternion cam_q{1.0, 0.0, 0.0, 0.0};
//   double **landmarks = (double **) malloc(sizeof(double **) * 5);
//   landmarks[0] = (double *) malloc(sizeof(double *) * 3);
//   landmarks[1] = (double *) malloc(sizeof(double *) * 3);
//   landmarks[2] = (double *) malloc(sizeof(double *) * 3);
//   landmarks[3] = (double *) malloc(sizeof(double *) * 3);
//   landmarks[4] = (double *) malloc(sizeof(double *) * 3);
//
//   // test and assert
//   BundleAdjustment ba;
//   ba.addCamera(K, features, landmark_ids, cam_t, cam_q, landmarks);
//
//   // clean up
//   free(landmarks[0]);
//   free(landmarks[1]);
//   free(landmarks[2]);
//   free(landmarks[3]);
//   free(landmarks[4]);
//   free(landmarks);
// }

TEST(BundleAdjustment, solve) {
    // create vo dataset
    VOTestDatasetGenerator generator;
    generator.configure(TEST_CONFIG);
    auto dataset = generator.generate();
    double **landmarks = build_landmark_matrix(dataset);

    // build bundle adjustment problem
    BundleAdjustment ba;
    std::vector<double *> translations;
    std::vector<double *> rotations;

    for (size_t i = 0; i < dataset.states.size(); i++) {
        // translation
        const auto &true_G_p_C_G = dataset.states[i].robot_G_p_B_G;
        // add noise
        Vec3 noisy_G_p_C_G = true_G_p_C_G + 0.1 * Vec3::Random();

        double *t_vec = (double *) malloc(sizeof(double *) * 3);
        t_vec[0] = noisy_G_p_C_G(0);
        t_vec[1] = noisy_G_p_C_G(1);
        t_vec[2] = noisy_G_p_C_G(2);
        translations.push_back(t_vec);

        // get rotation as quaternion
        const auto &initial_q = dataset.states[i].robot_q_GB;

        // @todo: add some noise
        auto noisy_q = initial_q;


        // Transform from NWU to EDN
        // This involves the rotation sequence: -90 deg about initial x axis,
        // 0, then -90 deg about initial z axis.
        Quaternion q_transform =
          Quaternion{Eigen::AngleAxisd(-M_PI_2, Vec3::UnitZ()) *
                     Eigen::AngleAxisd(-M_PI_2, Vec3::UnitX())};

        Quaternion q = q_transform * noisy_q;

        double *q_vec = (double *) malloc(sizeof(double *) * 4);
        q_vec[0] = q.w();
        q_vec[1] = q.x();
        q_vec[2] = q.y();
        q_vec[3] = q.z();
        rotations.push_back(q_vec);

        // add camera
        VecX landmark_ids =
          build_landmark_ids(dataset.states[i].features_observed);
        MatX features =
          build_feature_matrix(dataset.states[i].features_observed);
        ba.addCamera(
          dataset.camera_K, features, landmark_ids, t_vec, q_vec, landmarks);
    }

    // test
    ba.solve();

    // clean up
    free_2darray(landmarks, dataset.landmarks.size());
    for (auto t : translations) {
        free(t);
    }
    for (auto r : rotations) {
        free(r);
    }
}

}  // namespace wave
