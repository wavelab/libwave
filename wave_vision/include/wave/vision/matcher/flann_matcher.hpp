/**
 * @file
 * FLANN (Fast Library for Approximate Nearest Neighbours) based matcher
 * implementation, derived from descriptor matcher base class.
 * @ingroup vision
 */
#ifndef WAVE_VISION_FLANN_MATCHER_HPP
#define WAVE_VISION_FLANN_MATCHER_HPP

#include <string>
#include <vector>

#include <wave/vision/matcher/descriptor_matcher.hpp>

namespace wave {
/** @addtogroup vision
 *  @{ */

namespace FLANN {
/** The methods available in OpenCV to perform matching using FLANN. Options:
 *
 *  KDTree: The default method. This uses parallel kd-trees to separate
 *  keypoint descriptors, and  can then search the tree in order to determine
 *  the closest match.
 *
 *  KMeans: This method uses k-means clustering to sort the descriptors. The
 *  clusters are recursively optimized for a set number of iterations.
 *
 *  Composite: This method combines the above two methods, and aims to determine
 *  the optimum match between the two methods.
 *
 *  LSH: LSH stands for Locality-Sensitive Hash. This method uses hash functions
 *  to separate the descriptors, which allows for fast classification. The
 *  technique was proposed by [Lv et. al (2007)][LSH].
 *
 *  Autotuned: The last method looks to choose the optimal index method
 *  (KDTree, KMeans, LSH, etc.) based on defined optimality criteria.
 *
 *  For further reference on the different methods, please refer to pages
 *  575-580 of Kaehler and Bradski's book "Learning OpenCV 3: Computer Vision in
 *  C++ with the OpenCV Library".
 *
 *  [LSH]: http://www.cs.princeton.edu/cass/papers/mplsh_vldb07.pdf
 */
enum FLANNMethod {
    KDTree = 1,
    KMeans = 2,
    Composite = 3,
    LSH = 4,
    Autotuned = 5,
};
}

/** Representation of a descriptor matcher using the FLANN algorithm.
 *
 *  Internally, this class is wrapping OpenCV's FLANNBasedMatcher module.
 *  Further reference on the BFMatcher can be found
 * [here][opencv_flannmatcher].
 *
 *  Currently, this class only allows for the default parameters to be used for
 *  the selected method. TODO: Extend this to have customizable params.
 *
 *  [opencv_flannmatcher]:
 *  http://docs.opencv.org/trunk/dc/de2/classcv_1_1FlannBasedMatcher.html
 */
class FLANNMatcher : public DescriptorMatcher {
 public:
    /** Default constructor.
     *
     *  The default constructor will create the cv::FlannBasedMatcher object
     *  using the default parameters of the selected method. If no method is
     *  selected, the default is the KDTree.
     *
     * @param flann_method the search method to be used.
     */
    explicit FLANNMatcher(FLANN::FLANNMethod flann_method = FLANN::KDTree);

    /** Returns the current method being used by the FLANNMatcher.
     *
     * @return the method being used by the FLANNMatcher.
     */
    wave::FLANN::FLANNMethod getMethod() const {
        return this->curr_method;
    }

    /** Remove outliers between matches using epipolar constraints
     *
     * @param matches the unfiltered matches computed from two images
     * @param keypoints_1 the keypoints from the first image
     * @param keypoints_2 the keypoints from the second image
     *
     * @return the filtered matches
     */
    std::vector<cv::DMatch> removeOutliers(
      const std::vector<cv::DMatch> &matches,
      const std::vector<cv::KeyPoint> &keypoints_1,
      const std::vector<cv::KeyPoint> &keypoints_2) const override;

    /** Matches keypoints descriptors between two images using the
     *  FLANNMatcher.
     *
     *  @param descriptors_1 the descriptors extracted from the first image.
     *  @param descriptors_2 the descriptors extracted from the second image.
     *  @param keypoints_1 the keypoints detected in the first image
     *  @param keypoints_2 the keypoints detected in the second image
     *  @param mask
     *  \parblock indicates which descriptors can be matched between the two
     *  sets. As per OpenCV docs "queryDescriptors[i] can be matched with
     *  trainDescriptors[j] only if masks.at<uchar>(i,j) is non-zero. In the
     *  libwave wrapper, queryDescriptors are descriptors_1, and
     *  trainDescriptors are descriptors_2. Default is cv::noArray().
     *  \endparblock
     *
     *  @return vector containing the best matches.
     */
    std::vector<cv::DMatch> matchDescriptors(
      const cv::Mat &descriptors_1,
      const cv::Mat &descriptors_2,
      const std::vector<cv::KeyPoint> &keypoints_1,
      const std::vector<cv::KeyPoint> &keypoints_2,
      cv::InputArray mask = cv::noArray()) const override;

 private:
    /** First pass to filter bad matches. Takes in a vector of matches and uses
     *  the ratio test to filter the matches.
     *
     *  @param matches the unfiltered matches computed from two images.
     *
     *  @return the filtered matches.
     */
    std::vector<cv::DMatch> filterMatches(
            std::vector<std::vector<cv::DMatch>> &matches) const override;

    /** The pointer to the wrapped cv::FlannBasedMatcher object */
    cv::Ptr<cv::FlannBasedMatcher> flann_matcher;

    /** The configuration method currently being used */
    wave::FLANN::FLANNMethod curr_method;
};
}

#endif  // WAVE_VISION_FLANN_MATCHER_HPP
