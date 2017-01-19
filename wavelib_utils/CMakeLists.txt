CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_utils)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()


# PACKAGES
FIND_PACKAGE(GTest REQUIRED)


# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/include
    /usr/include/eigen3
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
    slam_utils
    STATIC
    src/data.cpp
    src/math.cpp
    src/time.cpp
    src/utils.cpp
)
SET(
    DEPS
    slam_utils
    ${GTEST_LIBRARIES}
    pthread
)


# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)

ADD_EXECUTABLE(data_test tests/data_test.cpp)
TARGET_LINK_LIBRARIES(data_test ${DEPS})

ADD_EXECUTABLE(math_test tests/math_test.cpp)
TARGET_LINK_LIBRARIES(math_test ${DEPS})

# ADD_EXECUTABLE(time_test tests/time_test.cpp)
# TARGET_LINK_LIBRARIES(time_test ${DEPS})

ADD_EXECUTABLE(utils_test tests/utils_test.cpp)
TARGET_LINK_LIBRARIES(utils_test ${DEPS})
