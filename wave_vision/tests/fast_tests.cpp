#include "wave/wave_test.hpp"
#include "wave/vision/fast_detector.hpp"

namespace wave {

#define TEST_CONFIG "config/fast_good.yaml"
#define TEST_IMAGE "data/lenna.png"

// Test fixture to load same image data
class FASTTest : public testing::Test {
 protected:
    FASTTest() {}
    virtual ~FASTTest() {
        if (this->detector) {
            delete this->detector;
        }
    }
    virtual void SetUp() {
        this->image = cv::imread(TEST_IMAGE, cv::IMREAD_COLOR);
    }

    void initDetector() {
        this->detector = new FASTDetector(TEST_CONFIG);
    }

    cv::Mat image;
    FASTDetector *detector;
};

TEST(FASTTests, GoodInitialization) {
    FASTDetector detector(TEST_CONFIG);
    SUCCEED();
}

}  // end of namespace wave