#ifndef __WAVE_MATCHING_MATCHER_HPP__
#define __WAVE_MATCHING_MATCHER_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace wave {
namespace matching {

// sensor data type
template <typename R>
class Matcher {
  public:
    // generic accessors
    const Eigen::Affine3d& get_result() {return result;};
    const Eigen::MatrixXd& get_info() {return information;};
    const float& getRes() {return resolution;};
    // Constructor with resolution
    Matcher(float res) : resolution(res) {}
    Matcher() {resolution = -1;}
    // intended to put sensor data into matcher class
    virtual void setRef(const R& ref) = 0;
    virtual void setTarget(const R& target) = 0;
    void setup(const R& ref, const R& target) {
        this->setRef(ref);
        this->setTarget(target);
    };
    // intended to actually perform the matching, and
    // then populate the private variables.
    // should return true if successful
    virtual bool match() {return 0;}

  protected:
    // internal storage
    float resolution;
    Eigen::Affine3d result;
    Eigen::MatrixXd information;
  private:
};

}  // end of matching namespace
}  // end of wave namespace
#endif
