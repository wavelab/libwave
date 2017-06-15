#ifndef ASLAM_CAMERAS_NO_MASK_HPP
#define ASLAM_CAMERAS_NO_MASK_HPP


namespace aslam {
namespace cameras {

class NoMask {
 public:

  enum {
    DesignVariableDimension = 0
  };

  NoMask();
  ~NoMask();

  template<typename K>
  bool isValid(const K & /* k */) const {
    return true;
  }

  // is the mask set? (i.e. mask data != NULL)
  bool isSet () const { return false; }

  static NoMask getTestMask() {
    return NoMask();
  }

};

}  // namespace cameras
}  // namespace aslam

#endif /* ASLAM_CAMERAS_NO_MASK_HPP */
