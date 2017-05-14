#ifndef WAVE_TEST_HPP
#define WAVE_TEST_HPP

#include <random>
#include <fstream>
#include <iostream>

#include <gtest/gtest.h>

#ifdef TEST_OUTPUT_ON
#define TEST_PRINT(M, ...) fprintf(stdout, M "\n", ##__VA_ARGS__)
#endif

#define TEST_RUNNER                                          \
    int main(int argc, char **argv) {                        \
        /* parse command line arguments */                   \
        for (int i = 0; i < argc; i++) {                     \
            if (strcmp(argv[i], "--silence-stdcout") == 0) { \
                std::cout.setstate(std::ios_base::failbit);  \
            }                                                \
        }                                                    \
        /* run tests */                                      \
        ::testing::InitGoogleTest(&argc, argv);              \
        return RUN_ALL_TESTS();                              \
    }

#endif
