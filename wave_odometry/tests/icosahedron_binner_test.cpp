#include "wave/wave_test.hpp"
#include "wave/odometry/icosahedron_binner.hpp"

namespace wave {

TEST(IcosahedronBinnerTests, Constructor) {
    IcosahedronBinner binner;
    MatX vectors = binner.getBinVectors();
    std::vector<int> counts = binner.getBinCounters();

    EXPECT_EQ(counts.size(), static_cast<unsigned long>(12));

    for (const auto &count : counts) {
        EXPECT_TRUE(count == 0);
    }

    auto sum = vectors.sum();

    EXPECT_NEAR(sum, 0.0, 1e-8);
}

TEST(IcosahedronBinnerTests, clear) {
    IcosahedronBinner binner;

    std::vector<Vec3, Eigen::aligned_allocator<Vec3>> normals(10);
    for (auto &normal : normals) {
        normal.setRandom();
        normal.normalize();
        binner.bin(normal);
    }

    std::vector<int> counts = binner.getBinCounters();
    int cnt = 0;
    for (const auto &count : counts) {
        cnt += count;
    }
    EXPECT_EQ(cnt, 10);

    binner.clear();

    counts = binner.getBinCounters();
    cnt = 0;
    for (const auto &count : counts) {
        cnt += count;
    }
    EXPECT_EQ(cnt, 0);
}

TEST(IcosahedronBinnerTests, bin_no_limit) {
    IcosahedronBinner binner;
    std::vector<Vec3, Eigen::aligned_allocator<Vec3>> duplicates(10);

    int cnt = 1;
    for (auto &normal : duplicates) {
        normal << 0.0, 0.0, 1.0;
        EXPECT_EQ(binner.bin(normal), cnt);
        ++cnt;
    }
    Vec3 other;
    other << 1.0, 0.0, 0.0;

    EXPECT_EQ(binner.bin(other), 1);
}

TEST(IcosahedronBinnerTests, bin_limit) {
    IcosahedronBinner binner;
    const int limit = 4;
    std::vector<Vec3, Eigen::aligned_allocator<Vec3>> duplicates(10);

    int cnt = 0;
    for (auto &normal : duplicates) {
        normal << 0.0, 0.0, 1.0;
        if (cnt < limit) {
            EXPECT_TRUE(binner.bin(normal, limit));
        } else {
            EXPECT_FALSE(binner.bin(normal, limit));
        }
        ++cnt;
    }
}

}
