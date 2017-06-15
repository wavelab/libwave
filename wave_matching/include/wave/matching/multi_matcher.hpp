/** @file
 * @ingroup matching
 *
 * @defgroup multi_matcher
 * Class to hold multiple instances of a matcher and multithread matches
 */

#ifndef WAVE_MULTI_MATCHER_HPP
#define WAVE_MULTI_MATCHER_HPP

#include <thread>
#include <mutex>
#include <map>
#include "wave/utils/math.hpp"
#include "wave/matching/pcl_common.hpp"
#include "wave/matching/matcher.hpp"

namespace wave {
/** @addtogroup matching
 *  @{ */

/**
 * Class is templated for different matcher types
 * @tparam T matcher type
 */
template <typename T>
class MultiMatcher {
 public:
    MultiMatcher() : max_threads(2) {};
    MultiMatcher(int n_threads) : max_threads(n_threads) {};

    void insert(const PCLPointCloud &src, const PCLPointCloud &target);
    bool getResult(int id);
 private:
    const int max_threads;

    /** Function run by each worker thread
     * @param id pair of clouds to match
     */
    void doMatch(int id);

    std::map<int, std::pair<PCLPointCloud, PCLPointCloud>> input;
    std::map<int, std::pair<Eigen::Affine3d, Mat6>> output;
    std::mutex ip_mutex;
    std::mutex op_mutex;

    std::vector<std::shared_ptr<std::thread>> workers;

    std::vector<T> matchers;
};

}  // namespace wave

#endif //WAVE_MULTI_MATCHER_HPP
