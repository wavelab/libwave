#include <gtest/gtest.h>

int main(int argc, char **argv) {
  // parse command line arguments
  for (int i = 0; i < argc; i++) {
    if (strcmp(argv[i], "--silence-stdcout") == 0) {
      std::cout.setstate(std::ios_base::failbit);
    }
  }

  // run tests
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
