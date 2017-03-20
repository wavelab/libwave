#ifndef WAVE_TEST_HPP
#define WAVE_TEST_HPP

#include <random>
#include <fstream>
#include <iostream>

#include <gtest/gtest.h>

#ifdef TEST_OUTPUT_ON
#define TEST_PRINT(M, ...) fprintf(stdout, M "\n", ##__VA_ARGS__)
#endif

#endif
