#ifndef ASLAM_CAMERAS_IMAGE_MASK_HPP
#define ASLAM_CAMERAS_IMAGE_MASK_HPP

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace aslam {
namespace cameras {

class ImageMask {
 public:
  ImageMask();
  ImageMask(const cv::Mat& mask, double scale = 1.0);
  ~ImageMask();

  void setMask(const cv::Mat& mask);
  const cv::Mat & getMask() const;

  void setScale(double scale);
  double getScale() const;

  // These guys mainly support the python interface
  void setMaskFromMatrix(const Eigen::MatrixXi & mask);
  Eigen::MatrixXi getMaskAsMatrix() const;

  template<typename DERIVED>
  bool isValid(const Eigen::MatrixBase<DERIVED> & k) const {
    int k1 = k(1, 0) * _scale;
    int k0 = k(0, 0) * _scale;
    // \todo fix this when it is initialized properly
    return !_mask.data || (_mask.at<unsigned char>(k1, k0) > 0);
    //return true;
  }

  // is the mask set? (i.e. mask data != NULL)
  bool isSet () const;

 private:
  cv::Mat _mask;
  double _scale;
};

}  // namespace cameras
}  // namespace aslam


#endif /* ASLAM_CAMERAS_IMAGE_MASK_HPP */
