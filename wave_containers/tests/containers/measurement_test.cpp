#include "wave/wave_test.hpp"

#include "wave/containers/measurement_container.hpp"
#include "wave/containers/measurement.hpp"

namespace wave {

enum class SomeSensors { S1, S2, S3 };

// This is the measurement type used in these tests
using TestMeasurement = Measurement<double, SomeSensors>;

TEST(Utils_measurement, insert) {
    MeasurementContainer<TestMeasurement> m;
    auto now = std::chrono::steady_clock::now();

    const auto
            meas = Measurement<double, SomeSensors>{now, SomeSensors::S1, 2.5};

    auto res = m.insert(meas);

    EXPECT_EQ(1ul, m.size());
    EXPECT_TRUE(res.second);

    // Insert the same thing
    auto res2 = m.insert(meas);
    EXPECT_FALSE(res2.second);
    EXPECT_EQ(res.first, res2.first);
}

TEST(Utils_measurement, emplace) {
    MeasurementContainer<TestMeasurement> m;
    auto now = std::chrono::steady_clock::now();

    auto res = m.emplace(now, SomeSensors::S1, 2.5);

    EXPECT_EQ(1ul, m.size());
    EXPECT_TRUE(res.second);

    // Insert the same thing
    auto res2 = m.emplace(now, SomeSensors::S1, 2.5);
    EXPECT_FALSE(res2.second);
    EXPECT_EQ(res.first, res2.first);

    std::vector<double> v;
}

TEST(Utils_measurement, capacity) {
    MeasurementContainer<TestMeasurement> m;
    EXPECT_EQ(0ul, m.size());
    EXPECT_TRUE(m.empty());

    m.emplace(std::chrono::steady_clock::now(), SomeSensors::S1, 2.5);
    EXPECT_EQ(1ul, m.size());
    EXPECT_FALSE(m.empty());
}

TEST(Utils_measurement, erase) {
    MeasurementContainer<TestMeasurement> m;

    auto now = std::chrono::steady_clock::now();
    m.emplace(now, SomeSensors::S1, 2.5);
    ASSERT_EQ(1ul, m.size());

    auto res = m.erase(now, SomeSensors::S2);
    EXPECT_EQ(0ul, res);
    EXPECT_EQ(1ul, m.size());

    res = m.erase(now, SomeSensors::S1);
    EXPECT_EQ(1ul, res);
    EXPECT_EQ(0ul, m.size());
}

TEST(Utils_measurement, get) {
    MeasurementContainer<TestMeasurement> m;

    auto now = std::chrono::steady_clock::now();
    auto value_in = 3.5;
    m.emplace(now, SomeSensors::S1, value_in);

    auto value_out = m.get(now, SomeSensors::S1);

    EXPECT_DOUBLE_EQ(value_in, value_out);
}


TEST(Utils_measurement, get_interpolated) {
    MeasurementContainer<TestMeasurement> m;
    auto t1 = std::chrono::steady_clock::now();
    auto t2 = t1 + std::chrono::seconds(10);
    auto tmid = t1 + std::chrono::seconds(5);

    auto v1 = 3.5, v2 = 8.0;
    m.emplace(t1, SomeSensors::S1, v1);
    m.emplace(t2, SomeSensors::S1, v2);

    // Measurements with different id - expect no effect
    m.emplace(tmid, SomeSensors::S2, -100.);
    m.emplace(t1 + std::chrono::seconds(6), SomeSensors::S2, -99.);

    auto value_out = m.get(tmid, SomeSensors::S1);

    auto expected = (v1 + v2) / 2.;
    EXPECT_DOUBLE_EQ(expected, value_out);
}


TEST(Utils_measurement, get_interpolated_other_sensors) {
    // Verify measurements from other sensors are NOT interpolated

    MeasurementContainer<TestMeasurement> m;
    auto t1 = std::chrono::steady_clock::now();
    auto t2 = t1 + std::chrono::seconds(10);
    auto tmid = t1 + std::chrono::seconds(5);

    auto v1 = 3.5, v2 = 8.0;
    m.emplace(t1, SomeSensors::S1, 10);
    m.emplace(t2, SomeSensors::S1, 20);
    m.emplace(t1, SomeSensors::S3, 70);
    m.emplace(t2, SomeSensors::S3, 80);

    EXPECT_THROW(m.get(tmid, SomeSensors::S2), std::out_of_range);
}

TEST(Utils_measurement, iterators) {
    // Todo: not sure how to check that all iterator functionality works
    // Since I'm just wrapping internal iterators, just do a simple sanity check
    MeasurementContainer<TestMeasurement> m;

    EXPECT_EQ(m.begin(), m.end());
    EXPECT_EQ(m.cbegin(), m.cend());

    for (auto i = 0; i < 5; ++i) {
        auto now = std::chrono::steady_clock::now();
        auto value_in = i * 1.1;
        m.emplace(now, SomeSensors::S1, i);
    }
    ASSERT_EQ(5ul, m.size());

    EXPECT_EQ(5, std::distance(m.begin(), m.end()));
    EXPECT_EQ(5, std::distance(m.cbegin(), m.cend()));

    auto expected = 0;
    // Note the container supports range-based for loops
    for (auto &v : m) {
        EXPECT_DOUBLE_EQ(expected, v.value);
        expected += 1.1;
    }

    const MeasurementContainer<TestMeasurement> cm;
    EXPECT_EQ(cm.begin(), cm.end());
}

TEST(Utils_measurement, clear) {
    MeasurementContainer<TestMeasurement> m;
    auto now = std::chrono::steady_clock::now();
    for (auto i = 0; i < 5; ++i) {
        m.emplace(now + std::chrono::seconds(i), SomeSensors::S1, 2.5);
    }
    ASSERT_EQ(5ul, m.size());

    m.clear();

    EXPECT_EQ(0ul, m.size());
    EXPECT_TRUE(m.empty());
}

}  // namespace wave
