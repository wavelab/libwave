#ifndef __WAVE_TEST_HPP__
#define __WAVE_TEST_HPP__

#include <random>
#include <fstream>
#include <iostream>

#include <gtest/gtest.h>

#ifdef TEST_OUTPUT_ON
#define TEST_PRINT(M, ...) fprintf(stdout, M "\n", ##__VA_ARGS__)
#endif

#endif
