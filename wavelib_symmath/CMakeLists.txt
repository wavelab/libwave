CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_symmath)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()


# PACKAGES
FIND_PACKAGE(GTest REQUIRED)


# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/include/
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
    slam_symmath
    STATIC
    src/models.cpp
    src/symmath.cpp
)
SET(
    DEPS
    slam_symmath
    ginac
    ${GTEST_LIBRARIES}
    pthread
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

ADD_EXECUTABLE(models_test tests/models_test.cpp)
TARGET_LINK_LIBRARIES(models_test ${DEPS})

ADD_EXECUTABLE(symmath_test tests/symmath_test.cpp)
TARGET_LINK_LIBRARIES(symmath_test ${DEPS})
