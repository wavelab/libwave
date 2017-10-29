PROJECT(wave_vision)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    wave::kinematics
    wave::utils
    wave::containers
    Eigen3::Eigen
    Boost::filesystem
    opencv_core opencv_features2d opencv_calib3d opencv_imgproc opencv_highgui
    SOURCES
    src/utils.cpp
    src/dataset/VoDataset.cpp
    src/dataset/VoTestCamera.cpp
    src/detector/fast_detector.cpp
    src/detector/orb_detector.cpp
    src/descriptor/brisk_descriptor.cpp
    src/descriptor/orb_descriptor.cpp
    src/matcher/brute_force_matcher.cpp
    src/matcher/flann_matcher.cpp)

# Unit tests
WAVE_ADD_TEST(${PROJECT_NAME}_tests
              tests/detector_tests/fast_tests.cpp
              tests/detector_tests/orb_tests.cpp
              tests/descriptor_tests/brisk_tests.cpp
              tests/descriptor_tests/orb_tests.cpp
              tests/matcher_tests/brute_force_tests.cpp
              tests/matcher_tests/flann_tests.cpp
              tests/tracker_tests/tracker_tests.cpp
              tests/dataset_tests/vo_dataset_tests.cpp
              tests/utils_tests.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})

WAVE_ADD_TEST(
    ${PROJECT_NAME}_viz_tests
    tests/viz_tests/detector_viz_tests.cpp
    tests/viz_tests/descriptor_viz_tests.cpp
    tests/viz_tests/matcher_viz_tests.cpp
    tests/viz_tests/tracker_viz_tests.cpp
    DISABLED # Requires display to run
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_viz_tests ${PROJECT_NAME})

# COPY TEST DATA
FILE(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)
