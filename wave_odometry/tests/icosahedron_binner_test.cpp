#include "wave/wave_test.hpp"
#include "wave/odometry/icosahedron_binner.hpp"
#include "wave/utils/config.hpp"

namespace {

void loadBinnerParams(const std::string &dir, const std::string &filename, wave::IcosahedronBinnerParams &params) {
    wave::ConfigParser parser;

    parser.addParam("range_divisions", &(params.range_divisions));
    parser.addParam("azimuth_divisions", &(params.azimuth_divisions));
    parser.addParam("xy_directions", &(params.xy_directions));
    parser.addParam("z_cutoff", &(params.z_cutoff));
    parser.addParam("z_limit", &(params.z_limit));
    parser.addParam("xy_limit", &(params.xy_limit));

    parser.load(dir + filename);
}

}

namespace wave {

// Fixture to perform setup
class BinnerTests : public testing::Test {
 protected:
    BinnerTests() {}

    virtual ~BinnerTests() {}

    virtual void SetUp() {
        loadBinnerParams("config/", "bin_config.yaml", this->params);
    }

    IcosahedronBinnerParams params;
};

TEST_F(BinnerTests, Constructor) {
    IcosahedronBinner binner(this->params);
}

TEST_F(BinnerTests, clear) {
    this->params.z_limit = 10;
    this->params.xy_limit = 10;
    IcosahedronBinner binner(this->params);

    std::vector<Vec6, Eigen::aligned_allocator<Vec6>> vectors(10);
    for (auto &normal : vectors) {
        normal.setRandom();
        normal.block<3,1>(0,0).normalize();
        binner.bin(normal);
    }

    auto counts = binner.getBinCounters();

    Eigen::Tensor<int, 0> sum = counts.sum();

    EXPECT_EQ(sum(), 10);

    binner.clear();

    counts = binner.getBinCounters();
    sum = counts.sum();
    EXPECT_EQ(sum(), 0);
}

TEST_F(BinnerTests, bin_no_limit) {
    this->params.z_limit = 100;
    IcosahedronBinner binner(this->params);
    std::vector<Vec6, Eigen::aligned_allocator<Vec6>> duplicates(10);

    int cnt = 1;
    int bin_cnt;
    for (auto &normal : duplicates) {
        normal << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
        binner.bin(normal, &bin_cnt);
        EXPECT_EQ(bin_cnt, cnt);
        ++cnt;
    }
}

TEST_F(BinnerTests, bin_limit) {
    const int limit = 4;
    this->params.z_limit = limit;
    IcosahedronBinner binner(this->params);

    std::vector<Vec6, Eigen::aligned_allocator<Vec6>> duplicates(10);

    int cnt = 0;
    for (auto &normal : duplicates) {
        normal << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
        if (cnt < limit) {
            EXPECT_TRUE(binner.bin(normal));
        } else {
            EXPECT_FALSE(binner.bin(normal));
        }
        ++cnt;
    }
}

}
