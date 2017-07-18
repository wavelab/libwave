#include "wave/wave_test.hpp"

#include "wave/containers/measurement_container.hpp"
#include "wave/containers/measurement.hpp"

namespace wave {

enum class SomeSensors { S1, S2, S3 };

// This is the measurement type used in these tests
using TestMeasurement = Measurement<double, SomeSensors>;

using std::chrono::seconds;

TEST(Utils_measurement, insert) {
    MeasurementContainer<TestMeasurement> m;
    auto now = std::chrono::steady_clock::now();

    const auto meas =
      Measurement<double, SomeSensors>{now, SomeSensors::S1, 2.5};

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
}

TEST(Utils_measurement, capacity) {
    MeasurementContainer<TestMeasurement> m;
    EXPECT_EQ(0ul, m.size());
    EXPECT_TRUE(m.empty());

    m.emplace(std::chrono::steady_clock::now(), SomeSensors::S1, 2.5);
    EXPECT_EQ(1ul, m.size());
    EXPECT_FALSE(m.empty());
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
    auto t2 = t1 + seconds(10);
    auto tmid = t1 + seconds(5);

    auto v1 = 3.5, v2 = 8.0;
    m.emplace(t1, SomeSensors::S1, v1);
    m.emplace(t2, SomeSensors::S1, v2);

    // Measurements with different id - expect no effect
    m.emplace(tmid, SomeSensors::S2, -100.);
    m.emplace(t1 + seconds(6), SomeSensors::S2, -99.);

    auto value_out = m.get(tmid, SomeSensors::S1);

    auto expected = (v1 + v2) / 2.;
    EXPECT_DOUBLE_EQ(expected, value_out);
}


TEST(Utils_measurement, get_interpolated_other_sensors) {
    // Verify measurements from other sensors are NOT interpolated

    MeasurementContainer<TestMeasurement> m;
    auto t1 = std::chrono::steady_clock::now();
    auto t2 = t1 + seconds(10);
    auto tmid = t1 + seconds(5);

    m.emplace(t1, SomeSensors::S1, 10.);
    m.emplace(t2, SomeSensors::S1, 20.);
    m.emplace(t1, SomeSensors::S3, 70.);
    m.emplace(t2, SomeSensors::S3, 80.);

    EXPECT_THROW(m.get(tmid, SomeSensors::S2), std::out_of_range);
}


TEST(Utils_measurement, iterators) {
    // Since I'm just wrapping internal iterators, just do a simple sanity check
    MeasurementContainer<TestMeasurement> m;

    EXPECT_EQ(m.begin(), m.end());
    EXPECT_EQ(m.cbegin(), m.cend());

    const MeasurementContainer<TestMeasurement> cm;
    EXPECT_EQ(cm.begin(), cm.end());
}

TEST(Utils_measurement, clear) {
    MeasurementContainer<TestMeasurement> m;
    auto now = std::chrono::steady_clock::now();
    for (auto i = 0; i < 5; ++i) {
        m.emplace(now + seconds(i), SomeSensors::S1, 2.5);
    }
    ASSERT_EQ(5ul, m.size());

    m.clear();

    EXPECT_EQ(0ul, m.size());
    EXPECT_TRUE(m.empty());
}

/** Test fixture with sample data */
class FilledMeasurementContainer : public ::testing::Test {
 protected:
    MeasurementContainer<TestMeasurement> m;
    // Define some sample input measurements
    const TimeType t_start = std::chrono::steady_clock::now();
    const std::vector<double> inputs = {1.2, 10, 3.4, 25, 5.6, -7, 7.8, 0};


    FilledMeasurementContainer() {
        for (int i = 0; i < 4; ++i) {
            auto t = this->t_start + seconds(i);
            m.emplace(t, SomeSensors::S1, this->inputs[2 * i]);
            m.emplace(t, SomeSensors::S2, this->inputs[2 * i + 1]);
        }
    }
};

TEST_F(FilledMeasurementContainer, eraseByKey) {
    // Erase non-existent key
    auto res = this->m.erase(this->t_start, SomeSensors::S3);
    EXPECT_EQ(0ul, res);
    EXPECT_EQ(this->inputs.size(), this->m.size());

    // Erase existent key
    res = this->m.erase(this->t_start, SomeSensors::S2);
    EXPECT_EQ(1ul, res);
    EXPECT_EQ(this->inputs.size() - 1, m.size());
}

TEST_F(FilledMeasurementContainer, eraseByPosition) {
    // Erase existent position
    auto b = this->m.begin();
    auto res = this->m.erase(b);
    EXPECT_EQ(++b, res);
    EXPECT_EQ(this->inputs.size() - 1, m.size());
    EXPECT_DOUBLE_EQ(this->inputs[1], this->m.begin()->value);

    // Erase last position
    auto end = this->m.end();
    res = this->m.erase(--end);
    EXPECT_EQ(this->m.end(), res);
    EXPECT_EQ(this->inputs.size() - 2, m.size());
}

TEST_F(FilledMeasurementContainer, eraseByRange) {
    auto a = std::next(this->m.begin(), 1);
    auto b = std::next(this->m.begin(), 3);
    auto res = this->m.erase(a, b);
    EXPECT_EQ(b, res);
    EXPECT_EQ(this->inputs.size() - 2, m.size());
    EXPECT_DOUBLE_EQ(this->inputs[0], this->m.begin()->value);
    EXPECT_DOUBLE_EQ(this->inputs[3], b->value);
}

TEST_F(FilledMeasurementContainer, iterators) {
    // Since I'm just wrapping internal iterators, just do a simple sanity check
    ASSERT_EQ(8ul, this->m.size());

    EXPECT_EQ(8, std::distance(this->m.begin(), this->m.end()));
    EXPECT_EQ(8, std::distance(this->m.cbegin(), this->m.cend()));

    // Note the container supports range-based for loops
    auto i = 0;
    for (auto &v : this->m) {
        EXPECT_DOUBLE_EQ(this->inputs[i++], v.value);
    }
}

TEST_F(FilledMeasurementContainer, constructFromRange) {
    auto first = this->m.begin();
    auto last = this->m.end();
    auto m2 = MeasurementContainer<TestMeasurement>(first, last);

    // Check that the copy has the same content as the original
    ASSERT_EQ(this->m.size(), m2.size());
    auto it = m2.begin();
    while (it != m2.end()) {
        EXPECT_EQ(first++->value, it++->value);
    }

    // Check that the copy is independent of the original
    this->m.erase(this->t_start, SomeSensors::S1);
    EXPECT_NE(this->m.size(), m2.size());
}

TEST_F(FilledMeasurementContainer, insertRange) {
    auto a = std::next(this->m.begin(), 1);
    auto b = std::next(this->m.begin(), 3);

    auto m2 = MeasurementContainer<TestMeasurement>{};
    m2.insert(a, b);
    EXPECT_EQ(2ul, m2.size());

    auto it = m2.begin();
    EXPECT_DOUBLE_EQ(this->inputs[1], it->value);
    EXPECT_DOUBLE_EQ(this->inputs[2], (++it)->value);

    m2.insert(a, ++b);
    EXPECT_EQ(3ul, m2.size());
}

TEST_F(FilledMeasurementContainer, getTimeWindowEmpty) {
    // Check case of window with no measurements
    const auto t = this->t_start;
    auto res = this->m.getTimeWindow(t - seconds(2), t - seconds(1));
    EXPECT_EQ(res.first, res.second);
}

TEST_F(FilledMeasurementContainer, getTimeWindowBackwards) {
    // Check case of backwards window
    const auto t = this->t_start;
    auto res = this->m.getTimeWindow(t + seconds(10), t);
    EXPECT_EQ(res.first, res.second);
}

TEST_F(FilledMeasurementContainer, getTimeWindowAll) {
    // Check case of window containing all measurements
    const auto t = this->t_start;
    auto res = this->m.getTimeWindow(t, t + seconds(99));
    EXPECT_EQ(static_cast<signed>(this->m.size()),
              std::distance(res.first, res.second));

    for (int i = 0; res.first != res.second; ++i, ++res.first) {
        EXPECT_DOUBLE_EQ(this->inputs[i], res.first->value);
    }
}

TEST_F(FilledMeasurementContainer, getTimeWindowSome) {
    // Check case of window containing some measurements
    const auto t = this->t_start;
    auto res = this->m.getTimeWindow(t + seconds(1), t + seconds(2));
    EXPECT_EQ(4u, std::distance(res.first, res.second));

    const auto expected = std::vector<double>{3.4, 25, 5.6, -7};

    for (int i = 0; res.first != res.second; ++i, ++res.first) {
        EXPECT_DOUBLE_EQ(expected[i], res.first->value);
    }
}

TEST_F(FilledMeasurementContainer, getTimeWindowInstant) {
    // Check case of window where `start == end`
    const auto t = this->t_start + seconds(1);
    auto res = this->m.getTimeWindow(t, t);
    ASSERT_EQ(2u, std::distance(res.first, res.second));
    EXPECT_DOUBLE_EQ(3.4, res.first->value);
    EXPECT_DOUBLE_EQ(25., (++res.first)->value);
}

TEST_F(FilledMeasurementContainer, getAllFromSensor) {
    // Check case of empty result
    auto res = this->m.getAllFromSensor(SomeSensors::S3);
    EXPECT_EQ(res.first, res.second);

    // Check case of filled result
    res = this->m.getAllFromSensor(SomeSensors::S1);
    EXPECT_EQ(4, std::distance(res.first, res.second));
    const auto expected = std::vector<double>{1.2, 3.4, 5.6, 7.8};
    for (int i = 0; res.first != res.second; ++i, ++res.first) {
        EXPECT_DOUBLE_EQ(expected[i], res.first->value);
    }
}

}  // namespace wave
