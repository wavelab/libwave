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

/** The FLANN namespace contains the configuration parameters for the FLANN
 *  matcher. In addition to selecting the manner in which FLANN matching is to
 *  be performed, these methods can also be configured.
 */
namespace FLANN {
/** There are several methods available in OpenCV to perform matching using
 *  FLANN. Currently, only the KDTree, KMeans, and Composite algorithms have
 *  been implemented. TODO: Add the remaining algorithms (Autotuned,
 *  Heirarchical Clustering, etc.).
 *
 *  KDTree: The default method. This uses parallel kd-trees to separate
 *  keypoint descriptors, and  can then search the tree in order to determine
 *  the closest match.
 *
 *  KMeans: This method uses k-means clustering to sort the descriptors. The
 *  clusters are recursively optimized for a set number of iterations.
 *
 *  Composite: This method combines the above two methods, and aims to determine
 *  the optimum match between the two.
 *
 *  LSH: Locality-Senstive hashing (LSH) uses hash functions to distribute the
 *  descriptors into similar buckets. The speed of hash tables allows for
 *  candidate matches to be generated very quickly. This was proposed by [Lv et.
 *  al (2007)][LSH].
 *
 *  These brief descriptions were adapted from Kaehler and Bradski's book
 *  "Learning OpenCV 3: Computer Vision in C++ with the OpenCV Library". For
 *  further reference on the different methods, please refer to pages 575-580.
 *
 *  [LSH]: http://www.cs.princeton.edu/cass/papers/mplsh_vldb07.pdf
 */
enum { KDTree = 1, KMeans = 2, Composite = 3, LSH = 4 };
}

struct FLANNMatcherParams {
    FLANNMatcherParams() {}

    /** Constructor with user selected values. Only to be used if the user
     *  desires the ratio test as the first pass to filter outliers.
     */
    FLANNMatcherParams(int flann_method,
                       double ratio_threshold,
                       bool auto_remove_outliers,
                       int fm_method)
        : flann_method(flann_method),
          use_knn(true),
          ratio_threshold(ratio_threshold),
          auto_remove_outliers(auto_remove_outliers),
          fm_method(fm_method) {}

    /** Overloaded method. Only to be used if the user desires the distance
     *  threshold test as the first pass to filter outliers.
     */
    FLANNMatcherParams(int flann_method,
                       int distance_threshold,
                       bool auto_remove_outliers,
                       int fm_method)
        : flann_method(flann_method),
          use_knn(false),
          distance_threshold(distance_threshold),
          auto_remove_outliers(auto_remove_outliers),
          fm_method(fm_method) {}

    /** Constructor using parameters extracted from a configuration file.
     *
     *  @param config_path the path to the location of the configuration file.
     */
    FLANNMatcherParams(const std::string &config_path);

    /** The FLANN method to use (described in the FLANN namespace). As a note,
     *  currently selecting a method will set up the FLANN matcher with
     *  default parameters.
     *
     *  Options:
     *  FLANN::KDTree: kd-tree separation
     *  FLANN::KMeans: k-means clustering.
     *  FLANN::Composite: Combines the above methods.
     *  FLANN::LSH: Locality-sensitive hash table separation.
     *
     *  Recommended: FLANN::KDTree
     */
    int flann_method = FLANN::KDTree;

    /** Determines whether to use a k-nearest-neighbours match.
     *
     *  Matcher can conduct a knn match with the best 2 matches for each
     *  descriptor. This uses the ratio test (@param ratio_threshold)
     *  to discard outliers.
     *
     *  If false, the matcher uses a distance heuristic
     *  (@param distance_threshold) to discard poor matches. This also
     *  incorporates cross checking between matches.
     *
     *  Recommended: true.
     */
    bool use_knn = true;

    /** Specifies heuristic for the ratio test, illustrated by Dr. David G. Lowe
     *  in his paper _Distinctive Image Features from Scale-Invariant Keypoints_
     *  (2004). The test takes the ratio of the closest keypoint distance
     *  to that of the second closest neighbour. If the ratio is less than
     *  the heuristic, it is discarded.
     *
     *  A value of 0.8 was shown by Dr. Lowe to reject 90% of the false matches,
     *  and discard only 5% of the correct matches.
     *
     *  Recommended: 0.8. Must be between 0 and 1.
     */
    double ratio_threshold = 0.8;

    /** Specifies the distance threshold for good matches.
     *
     *  Matches will only be kept if the descriptor distance is less than or
     *  equal to the product of the distance threshold and the _minimum_ of all
     *  descriptor distances. The greater the value, the more matches will
     *  be kept.
     *
     *  Recommended: 5. Must be greater than or equal to zero.
     */
    int distance_threshold = 5;

    /** Determines whether to automatically remove outliers using the method
     *  described in fm_method.
     *
     *  If true, the wave::BruteForceMatcher::matchDescriptors method will
     *  automatically call the wave::BruteForceMatcher::removeOutliers method,
     *  which uses the method in wave::BFMatcherParams::fm_method to remove
     *  matched outliers.
     *
     *  If false, the matches returned will only have been passed through the
     *  distance threshold or ratio tests described above.
     *
     *  Recommended: True
     */
    bool auto_remove_outliers = true;

    /** Method to find the fundamental matrix and remove outliers.
     *
     *  Options:
     *  cv::FM_7POINT: 7-point algorithm
     *  cv::FM_8POINT: 8-point algorithm
     *  cv::FM_LMEDS : least-median algorithm
     *  cv::FM_RANSAC: RANSAC algorithm
     *
     *  Recommended: cv::FM_RANSAC.
     */
    int fm_method = cv::FM_RANSAC;
};

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
    // Diagnostics information
    /** The number of matches prior to any filtering or outlier removal. */
    size_t num_raw_matches = 0u;

    /** The number of matches after being filtered by the distance/ratio test.
     *  This is the input to the outlierRejection method.
     */
    size_t num_filtered_matches = 0u;

    /** The size of matches remaining after outlier removal via RANSAC or a
     *  similar method.
     */
    size_t num_good_matches = 0u;

    /** Default constructor. The user can also specify their own struct with
     *  desired values. If no struct is provided, default values are used.
     *
     *  @param config contains the desired parameter values.
     */
    explicit FLANNMatcher(
      const FLANNMatcherParams &config = FLANNMatcherParams{});

    /** Returns the current configuration parameters being used by the
     *  FLANNMatcher
     *
     *  @return the current configuration values.
     */
    FLANNMatcherParams getConfiguration() const {
        return this->current_config;
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
      cv::Mat &descriptors_1,
      cv::Mat &descriptors_2,
      const std::vector<cv::KeyPoint> &keypoints_1,
      const std::vector<cv::KeyPoint> &keypoints_2,
      cv::InputArray mask = cv::noArray()) override;

 private:
    /** The pointer to the wrapped cv::FlannBasedMatcher object */
    cv::Ptr<cv::FlannBasedMatcher> flann_matcher;

    /** Current configuration parameters*/
    FLANNMatcherParams current_config;

    /** Checks whether the desired configuration is valid
     *
     *  @param check_config containing the desired configuration values.
     */
    void checkConfiguration(const FLANNMatcherParams &check_config);

    /** Remove outliers between matches. Uses a heuristic based approach as a
     *  first pass to determine good matches.
     *
     *  @param matches the unfiltered matches computed from two images.
     *
     *  @return the filtered matches.
     */
    std::vector<cv::DMatch> filterMatches(
      std::vector<cv::DMatch> &matches) const override;

    /** First pass to filter bad matches. Takes in a vector of matches and uses
     *  the ratio test to filter the matches.
     *
     *  @param matches the unfiltered matches computed from two images.
     *
     *  @return the filtered matches.
     */
    std::vector<cv::DMatch> filterMatches(
      std::vector<std::vector<cv::DMatch>> &matches) const override;
};
}  // namespace wave

#endif  // WAVE_VISION_FLANN_MATCHER_HPP
