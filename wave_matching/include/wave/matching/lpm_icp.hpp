#ifndef WAVE_MATCHING_LPM_ICP_HPP
#define WAVE_MATCHING_LPM_ICP_HPP

#include <string>
#include <pointmatcher/PointMatcher.h>

#include "wave/matching/matcher.hpp"

namespace wave {

struct LPMMatcher : public Matcher<boost::shared_ptr<PointMatcher<double>::DataPoints>> {
    public:
        explicit LPMMatcher(std::string yaml_file);

        ~LPMMatcher();

        void setRef(const boost::shared_ptr<PointMatcher<double>::DataPoints> &ref);

        void setTarget(const boost::shared_ptr<PointMatcher<double>::DataPoints> &target);

        bool match();

        // TODO(jskhu): implement this when it's ready
        void estimateInfo();
    
    private:
        PointMatcher<double>::ICP icp;
        boost::shared_ptr<PointMatcher<double>::DataPoints> ref, target, final, downsampled_ref,
            downsampled_target;
        PointMatcher<double>::TransformationParameters transform_params;
};

} // namespace wave
#endif // WAVE_MATCHING_LPM_ICP_HPP