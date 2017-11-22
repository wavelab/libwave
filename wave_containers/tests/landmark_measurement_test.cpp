#include "wave/wave_test.hpp"

#include "wave/containers/landmark_measurement_container.hpp"
#include "wave/containers/landmark_measurement.hpp"

namespace wave {

using std::chrono::seconds;

enum class CameraSensors { Left = 0, Right = 1, Top = 2 };

// This is the measurement type used in these tests
using TestLandmarkMeasurement = LandmarkMeasurement<CameraSensors>;
using TestContainer = LandmarkMeasurementContainer<TestLandmarkMeasurement>;

TEST(LandmarkContainer, insert) {
    auto now = std::chrono::steady_clock::now();
    auto val = Vec2{1.2, 3.4};
    auto landmark_id = std::size_t{1};
    auto img = std::size_t{0};
    auto meas =
      TestLandmarkMeasurement{now, CameraSensors::Left, landmark_id, img, val};
    auto m = TestContainer{};

    auto res = m.insert(meas);

    EXPECT_EQ(1u, m.size());
    EXPECT_TRUE(res.second);

    // Insert the same thing
    auto res2 = m.insert(meas);
    EXPECT_FALSE(res2.second);
    EXPECT_EQ(res.first, res2.first);
}

TEST(LandmarkContainer, emplace) {
    auto m = TestContainer{};
    auto now = std::chrono::steady_clock::now();
    auto res = m.emplace(now, CameraSensors::Left, 1u, 0u, Vec2{1.2, 3.4});

    EXPECT_EQ(1u, m.size());
    EXPECT_TRUE(res.second);

    // Insert the same thing
    auto res2 = m.emplace(now, CameraSensors::Left, 1u, 0u, Vec2{2.3, 4.5});
    EXPECT_FALSE(res2.second);
    EXPECT_EQ(res.first, res2.first);
}

TEST(LandmarkContainer, capacity) {
    auto m = TestContainer{};
    auto now = std::chrono::steady_clock::now();

    EXPECT_EQ(0u, m.size());
    EXPECT_TRUE(m.empty());

    m.emplace(now, CameraSensors::Left, 1u, 0u, Vec2{1.2, 3.4});
    EXPECT_EQ(1u, m.size());
    EXPECT_FALSE(m.empty());
}

TEST(LandmarkContainer, erase) {
    auto m = TestContainer{};
    auto now = std::chrono::steady_clock::now();

    m.emplace(now, CameraSensors::Left, 1u, 0u, Vec2{1.2, 3.4});
    ASSERT_EQ(1u, m.size());

    auto res = m.erase(now, CameraSensors::Right, 1u);
    EXPECT_EQ(0u, res);
    EXPECT_EQ(1u, m.size());

    res = m.erase(now, CameraSensors::Left, 1u);
    EXPECT_EQ(1u, res);
    EXPECT_EQ(0u, m.size());
}

TEST(LandmarkContainer, get) {
    auto m = TestContainer{};
    auto now = std::chrono::steady_clock::now();
    auto value_in = Vec2{1.2, 3.4};
    m.emplace(now, CameraSensors::Left, 1u, 0u, value_in);

    auto value_out = m.get(now, CameraSensors::Left, 1u);

    EXPECT_PRED2(VectorsNear, value_in, value_out);
}

TEST(LandmarkContainer, iterators) {
    // Since I'm just wrapping internal iterators, just do a simple sanity check
    auto m = TestContainer{};

    EXPECT_EQ(m.begin(), m.end());
    EXPECT_EQ(m.cbegin(), m.cend());

    for (auto i = 0; i < 5; ++i) {
        auto now = std::chrono::steady_clock::now();
        auto value_in = Vec2{i * 1.1, 0};
        auto img = std::size_t{0};
        m.emplace(now, CameraSensors::Left, 1u, img, value_in);
    }
    ASSERT_EQ(5ul, m.size());
    EXPECT_EQ(5, std::distance(m.begin(), m.end()));
    EXPECT_EQ(5, std::distance(m.cbegin(), m.cend()));

    auto expected = Vec2{0, 0};
    // Note the container supports range-based for loops
    for (auto &v : m) {
        EXPECT_PRED2(VectorsNear, expected, v.value);
        expected(0) += 1.1;
    }

    const auto cm = TestContainer{};
    EXPECT_EQ(cm.begin(), cm.end());
}

TEST(LandmarkContainer, clear) {
    auto m = TestContainer{};
    auto now = std::chrono::steady_clock::now();
    auto value_in = Vec2{1.2, 3.4};
    for (auto i = 0u; i < 5ul; ++i) {
        m.emplace(now, CameraSensors::Left, i, 0u, value_in);
    }
    ASSERT_EQ(5ul, m.size());

    m.clear();

    EXPECT_EQ(0u, m.size());
    EXPECT_TRUE(m.empty());
}


TEST(LandmarkContainer, getLandmarkIDs) {
    auto m = TestContainer{};
    auto ids = m.getLandmarkIDs();
    EXPECT_TRUE(ids.empty());
}

/** Test fixture with sample data */
class FilledLandmarkContainer : public ::testing::Test {
 protected:
    TestContainer m;
    // Define some sample input measurements
    const TimePoint t_start = std::chrono::steady_clock::now();
    const std::vector<std::vector<LandmarkId>> input_ids_l = {
      {2}, {2}, {3, 4, 2}, {2, 5, 3}, {6}, {1}, {3}};
    const std::vector<std::vector<LandmarkId>> input_ids_r = {
      {1}, {}, {4, 2}, {4, 5}, {25, 1}, {}, {4}};
    // generated sample values
    std::vector<std::map<LandmarkId, Vec2>> inputs_l{input_ids_l.size()};
    std::vector<std::map<LandmarkId, Vec2>> inputs_r{input_ids_r.size()};

    // values sorted as expected (time, then sensor, then landmark id)
    std::vector<Vec2> expected_values;
    std::vector<Vec2> input_values_l;
    std::vector<Vec2> input_values_r;
    FilledLandmarkContainer() {
        // Generate inputs and fill container. Follow a pattern for debugging
        assert(7ul == input_ids_r.size());
        assert(7ul == input_ids_r.size());
        // Insert the measurements out-of-order by time, to ensure methods
        // always return output sorted by time.
        auto insert_order = std::vector<int>{1, 5, 2, 6, 4, 0, 3};

        for (auto i = 0u; i < input_ids_l.size(); ++i) {
            auto k = insert_order[i];
            auto t = this->t_start + seconds(k);
            for (auto id : this->input_ids_l[k]) {
                Vec2 val = Vec2::Ones() * (k + id / 10.0);
                this->m.emplace(t, CameraSensors::Left, id, (size_t) k, val);
                this->inputs_l[k][id] = val;
            }
            for (auto id : this->input_ids_r[k]) {
                Vec2 val = Vec2::Ones() * (10 + k + id / 10.0);
                this->m.emplace(t, CameraSensors::Right, id, (size_t) k, val);
                this->inputs_r[k][id] = val;
            }
        }
        for (auto i = 0u; i < input_ids_l.size(); ++i) {
            // Generate expected values with expected ordering
            for (auto v : this->inputs_l[i]) {
                this->expected_values.push_back(v.second);
                this->input_values_l.push_back(v.second);
            }
            for (auto v : this->inputs_r[i]) {
                this->expected_values.push_back(v.second);
                this->input_values_r.push_back(v.second);
            }
        }
    }
};

TEST_F(FilledLandmarkContainer, emplace) {
    // Check emplacement that occurred in the test fixture constructor
    auto i = 0u;
    for (auto a = this->m.begin(); a != this->m.end(); ++a) {
        ASSERT_PRED2(VectorsNear, this->expected_values[i++], a->value);
    }
}

TEST_F(FilledLandmarkContainer, eraseByKey) {
    ASSERT_EQ(this->expected_values.size(), this->m.size());

    // Erase non-existent key
    auto res = this->m.erase(this->t_start, CameraSensors::Left, 99);
    EXPECT_EQ(0u, res);
    EXPECT_EQ(this->expected_values.size(), this->m.size());

    // Erase existent key
    res = this->m.erase(this->t_start, CameraSensors::Right, 1);
    EXPECT_EQ(1u, res);
    EXPECT_EQ(this->expected_values.size() - 1, m.size());
}

TEST_F(FilledLandmarkContainer, eraseByPosition) {
    // Erase existent position
    auto a = this->m.begin();
    auto res = this->m.erase(a);
    EXPECT_EQ(++a, res);
    EXPECT_EQ(this->expected_values.size() - 1, m.size());
    EXPECT_PRED2(VectorsNear, this->expected_values[1], this->m.begin()->value);

    // Erase last position
    auto end = this->m.end();
    res = this->m.erase(--end);
    EXPECT_EQ(this->m.end(), res);
    EXPECT_EQ(this->expected_values.size() - 2, m.size());
    auto last = std::prev(this->m.end());
    EXPECT_PRED2(VectorsNear,
                 this->expected_values[this->expected_values.size() - 2],
                 last->value);
}

TEST_F(FilledLandmarkContainer, eraseByRange) {
    auto a = std::next(this->m.begin(), 1);
    auto b = std::next(this->m.begin(), 3);

    auto res = this->m.erase(a, b);
    EXPECT_EQ(b, res);
    EXPECT_EQ(this->expected_values.size() - 2, m.size());
    EXPECT_PRED2(VectorsNear, this->expected_values[0], this->m.begin()->value);
    EXPECT_PRED2(VectorsNear, this->expected_values[3], b->value);
}

TEST_F(FilledLandmarkContainer, iterators) {
    auto expected_size = static_cast<signed>(this->expected_values.size());

    // Since I'm just wrapping internal iterators, just do a simple sanity check
    EXPECT_EQ(expected_size, std::distance(this->m.begin(), this->m.end()));
    EXPECT_EQ(expected_size, std::distance(this->m.cbegin(), this->m.cend()));

    // Note the container supports range-based for loops
    auto i = 0u;
    for (auto &v : this->m) {
        EXPECT_PRED2(VectorsNear, this->expected_values[i++], v.value);
    }
}

TEST_F(FilledLandmarkContainer, constructFromRange) {
    auto first = this->m.begin();
    auto last = this->m.end();
    auto m2 = TestContainer(first, last);

    // Check that the copy has the same content as the original
    ASSERT_EQ(this->m.size(), m2.size());
    auto it = m2.begin();
    while (it != m2.end()) {
        EXPECT_EQ(first++->value, it++->value);
    }

    // Check that the copy is independent of the original
    this->m.erase(this->t_start, CameraSensors::Right, 1);
    EXPECT_LT(this->m.size(), m2.size());
}

TEST_F(FilledLandmarkContainer, insertRange) {
    auto a = std::next(this->m.begin(), 1);
    auto b = std::next(this->m.begin(), 3);

    auto m2 = TestContainer{};
    m2.insert(a, b);
    EXPECT_EQ(2u, m2.size());

    auto it = m2.begin();
    EXPECT_PRED2(VectorsNear, this->expected_values[1], it->value);
    EXPECT_PRED2(VectorsNear, this->expected_values[2], (++it)->value);

    m2.insert(a, ++b);
    EXPECT_EQ(3u, m2.size());
}

TEST_F(FilledLandmarkContainer, getTimeWindowEmpty) {
    // Check case of window with no measurements
    const auto t = this->t_start;
    auto res = this->m.getTimeWindow(t - seconds(2), t - seconds(1));
    EXPECT_EQ(res.first, res.second);
}

TEST_F(FilledLandmarkContainer, getTimeWindowBackwards) {
    // Check case of backwards window
    const auto t = this->t_start;
    auto res = this->m.getTimeWindow(t + seconds(10), t);
    EXPECT_EQ(res.first, res.second);
}

TEST_F(FilledLandmarkContainer, getTimeWindowAll) {
    // Check case of window containing all measurements
    const auto t = this->t_start;
    auto res = this->m.getTimeWindow(t, t + seconds(99));
    EXPECT_EQ(static_cast<signed>(this->m.size()),
              std::distance(res.first, res.second));

    for (auto i = 0u; res.first != res.second; ++i, ++res.first) {
        EXPECT_PRED2(VectorsNear, this->expected_values[i], res.first->value);
    }
}

TEST_F(FilledLandmarkContainer, getTimeWindowSome) {
    // Check case of window containing some measurements
    // The expected values here are manually picked by inspecting the fixture

    const auto t = this->t_start;
    auto res = this->m.getTimeWindow(t + seconds(1), t + seconds(2));
    EXPECT_EQ(6, std::distance(res.first, res.second));

    auto exp_first = this->expected_values.begin() + 2;
    auto exp_last = exp_first + 6;
    const auto expected = std::vector<Vec2>(exp_first, exp_last);

    for (int i = 0; res.first != res.second; ++i, ++res.first) {
        EXPECT_PRED2(VectorsNear, expected[i], res.first->value);
    }
}

TEST_F(FilledLandmarkContainer, getTimeWindowInstant) {
    // Check case of window where `start == end`
    const auto t = this->t_start + seconds(1);
    auto res = this->m.getTimeWindow(t, t);

    // Just ensure the result is not empty, don't worry about values here
    EXPECT_EQ(1u, std::distance(res.first, res.second));
}

TEST_F(FilledLandmarkContainer, getLandmarkIDs) {
    auto ids = this->m.getLandmarkIDs();

    // These expected values were counted manually from above
    auto expected_ids = std::vector<LandmarkId>{1, 2, 3, 4, 5, 6, 25};
    ASSERT_EQ(expected_ids.size(), ids.size());
    for (auto i = 0u; i < expected_ids.size(); ++i) {
        EXPECT_EQ(expected_ids[i], ids[i]);
    }
}

TEST_F(FilledLandmarkContainer, getAllFromSensor) {
    // Check case of empty result
    auto res = this->m.getAllFromSensor(CameraSensors::Top);
    EXPECT_EQ(res.first, res.second);

    // Check case of filled result
    res = this->m.getAllFromSensor(CameraSensors::Right);
    EXPECT_EQ(static_cast<signed>(this->input_values_r.size()),
              std::distance(res.first, res.second));

    for (auto i = 0; res.first != res.second; ++res.first, ++i) {
        EXPECT_PRED2(VectorsNear, this->input_values_r[i], res.first->value);
    }
}

TEST_F(FilledLandmarkContainer, getLandmarkIdsInWindowEmpty) {
    // Check case of window with no measurements
    const auto t = this->t_start;
    auto ids = this->m.getLandmarkIDsInWindow(t - seconds(2), t - seconds(1));
    EXPECT_TRUE(ids.empty());
}

TEST_F(FilledLandmarkContainer, getLandmarkIdsInWindowBackwards) {
    const auto t = this->t_start;
    // Check case of backwards window
    auto ids = this->m.getLandmarkIDsInWindow(t + seconds(2), t);
    EXPECT_TRUE(ids.empty());
}

TEST_F(FilledLandmarkContainer, getLandmarkIdsInWindowAll) {
    // Check case of window containing all measurements
    const auto t = this->t_start;
    // These expected values were chosen manually from the fixture
    auto ids = this->m.getLandmarkIDsInWindow(t, t + seconds(10));

    auto expected_ids = std::vector<LandmarkId>{1, 2, 3, 4, 5, 6, 25};
    EXPECT_EQ(expected_ids.size(), ids.size());
    for (auto i = 0u; i < ids.size(); ++i) {
        EXPECT_EQ(expected_ids[i], ids[i]);
    }
}
TEST_F(FilledLandmarkContainer, getLandmarkIdsInWindowSome) {
    // Check case of window containing some measurements
    const auto t = this->t_start;
    auto ids = this->m.getLandmarkIDsInWindow(t + seconds(5), t + seconds(6));
    const auto expected2 = std::vector<LandmarkId>{1, 3, 4};
    EXPECT_EQ(expected2.size(), ids.size());
    for (auto i = 0u; i < ids.size(); ++i) {
        EXPECT_EQ(expected2[i], ids[i]);
    }
}

TEST_F(FilledLandmarkContainer, getLandmarkIdsInWindowInstant) {
    const auto t = this->t_start + seconds(3);
    // Check case of window where start == end.
    // We expect it to return the landmarks at that instant
    auto ids = this->m.getLandmarkIDsInWindow(t, t);
    const auto expected = std::vector<LandmarkId>{2, 3, 4, 5};
    ASSERT_EQ(expected.size(), ids.size());
    for (auto i = 0u; i < ids.size(); ++i) {
        EXPECT_EQ(expected[i], ids[i]);
    }
}

TEST_F(FilledLandmarkContainer, getTrackInWindowEmpty) {
    // Check case of window with no measurements
    const auto t = this->t_start;
    auto track = this->m.getTrackInWindow(
      CameraSensors::Right, 4, t - seconds(2), t - seconds(1));
    EXPECT_TRUE(track.empty());
}

TEST_F(FilledLandmarkContainer, getTrackInWindowBackwards) {
    const auto t = this->t_start;
    // Check case of backwards window
    auto track =
      this->m.getTrackInWindow(CameraSensors::Right, 4, t + seconds(10), t);
    EXPECT_TRUE(track.empty());
}

TEST_F(FilledLandmarkContainer, getTrackInWindowInstant) {
    const auto t = this->t_start + seconds(3);
    // Check case where start == end
    // We expect it to return a track of size 1, if there happens to be a
    // measurement at that time point.
    auto track = this->m.getTrackInWindow(CameraSensors::Right, 4, t, t);
    ASSERT_EQ(1u, track.size());
    EXPECT_EQ(t, track.front().time_point);
    EXPECT_EQ(CameraSensors::Right, track.front().sensor_id);
    EXPECT_EQ(4u, track.front().landmark_id);
}

TEST_F(FilledLandmarkContainer, getTrackInWindowAll) {
    // Check case of window containing all measurements
    const auto t = this->t_start;
    auto track =
      this->m.getTrackInWindow(CameraSensors::Right, 4, t, t + seconds(10));
    // These expected values were chosen manually from the fixture
    auto expected_times =
      std::vector<TimePoint>{t + seconds(2), t + seconds(3), t + seconds(6)};

    ASSERT_EQ(expected_times.size(), track.size());
    for (auto i = 0u; i < track.size(); ++i) {
        EXPECT_EQ(expected_times[i], track[i].time_point);
        EXPECT_EQ(CameraSensors::Right, track[i].sensor_id);
        EXPECT_EQ(4u, track[i].landmark_id);
    }
}

TEST_F(FilledLandmarkContainer, getTrackInWindowSome) {
    // Check case of window containing all measurements
    const auto t = this->t_start;
    auto track = this->m.getTrackInWindow(
      CameraSensors::Right, 4, t + seconds(3), t + seconds(4));

    // These expected values were chosen manually from the fixture
    auto expected_times = std::vector<TimePoint>{t + seconds(3)};

    ASSERT_EQ(expected_times.size(), track.size());
    for (auto i = 0u; i < track.size(); ++i) {
        EXPECT_EQ(expected_times[i], track[i].time_point);
        EXPECT_EQ(CameraSensors::Right, track[i].sensor_id);
        EXPECT_EQ(4u, track[i].landmark_id);
    }
}

TEST_F(FilledLandmarkContainer, getTrack) {
    const auto t = this->t_start;
    auto track = this->m.getTrack(CameraSensors::Right, 4);

    // This expected results are the same as getTrackInWindowAll
    // These expected values were chosen manually from the fixture
    auto expected_times =
      std::vector<TimePoint>{t + seconds(2), t + seconds(3), t + seconds(6)};

    ASSERT_EQ(expected_times.size(), track.size());
    for (auto i = 0u; i < track.size(); ++i) {
        EXPECT_EQ(expected_times[i], track[i].time_point);
        EXPECT_EQ(CameraSensors::Right, track[i].sensor_id);
        EXPECT_EQ(4u, track[i].landmark_id);
    }
}

TEST_F(FilledLandmarkContainer, getTrackEmpty) {
    // Check cases of a query with no measurements
    auto track = this->m.getTrack(CameraSensors::Top, 4);
    EXPECT_TRUE(track.empty());

    track = this->m.getTrack(CameraSensors::Left, 999);
    EXPECT_TRUE(track.empty());
}

}  // namespace wave
