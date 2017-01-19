CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_optimization)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# PACKAGES
FIND_PACKAGE(GTest REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)


# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/include/
    /usr/include/eigen3
    ${CERES_INCLUDE_DIRS}
    /usr/local/include
    include
)


# LIBS
LINK_DIRECTORIES(
    /usr/lib
    /usr/local/lib
)


# LIBRARY
ADD_LIBRARY(
    slam_optimization
    STATIC
    src/ceres/ba.cpp
    src/optimizers/gd.cpp
    src/optimizers/lls.cpp
    src/optimizers/lma.cpp
    src/optimizers/ransac.cpp
    src/benchmark.cpp
    src/testcase.cpp
)
SET(
    DEPS
    slam_optimization
    slam_utils
    ${CERES_LIBRARIES}
    ${GTEST_LIBRARIES}
    pthread
)



# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY scripts DESTINATION ${PROJECT_BINARY_DIR})

ADD_EXECUTABLE(ba_test tests/ceres/ba_test.cpp)
TARGET_LINK_LIBRARIES(ba_test ${DEPS})

ADD_EXECUTABLE(gd_test tests/optimizers/gd_test.cpp)
TARGET_LINK_LIBRARIES(gd_test ${DEPS})

ADD_EXECUTABLE(lls_test tests/optimizers/lls_test.cpp)
TARGET_LINK_LIBRARIES(lls_test ${DEPS})

ADD_EXECUTABLE(lma_test tests/optimizers/lma_test.cpp)
TARGET_LINK_LIBRARIES(lma_test ${DEPS})

ADD_EXECUTABLE(ransac_test tests/optimizers/ransac_test.cpp)
TARGET_LINK_LIBRARIES(ransac_test ${DEPS})

ADD_EXECUTABLE(testcase_test tests/testcase_test.cpp)
TARGET_LINK_LIBRARIES(testcase_test ${DEPS})
