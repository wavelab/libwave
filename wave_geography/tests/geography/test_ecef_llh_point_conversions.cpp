#include <gtest/gtest.h>
#include "wave/geography/world_frame_conversions.hpp"

namespace wave {

TEST(ecefAndLLHPointConversionTest, everestTest) {
    // From GeographicLib's example:
    // Mt Everest
    //      LLH[deg, deg, m] ~ (27.99, 86.93, 8820)
    //      ECEF[km] ~ (302.3, 5636, 2980)
    //      ECEF[m] ~ (3.023e5, 5.636e6, 2.980e6)

    Eigen::Vector3d point_llh(27.99, 86.93, 8820);
    Eigen::Vector3d point_ECEF(3.023e5, 5.636e6, 2.980e6);

    // LLH to ECEF
    Eigen::Vector3d point_ECEF_Results = ecefPointFromLLH(point_llh);

    // Precisions are based on the sig-fig data above -> therefore different
    // checks have different bounds based on their order of magnitude.
    ASSERT_NEAR(point_ECEF(0), point_ECEF_Results(0), 1e2);
    ASSERT_NEAR(point_ECEF(1), point_ECEF_Results(1), 1e3);
    ASSERT_NEAR(point_ECEF(2), point_ECEF_Results(2), 1e3);

    // ECEF back to LLH
    Eigen::Vector3d point_llh_results = llhPointFromECEF(point_ECEF_Results);

    ASSERT_NEAR(point_llh(0), point_llh_results(0), 1e-2);
    ASSERT_NEAR(point_llh(1), point_llh_results(1), 1e-2);
    ASSERT_NEAR(point_llh(2), point_llh_results(2), 1.0);
}

TEST(ecefAndLLHPointConversionTest, grandCanyonTest) {
    // Second test for a lower point located elsewhere:
    // Badwater Basin
    //      LLH[deg, deg, m] ~ (36.250278, -116.825833, -85.9536)
    //      ECEF[km] ~ (-2323.892, -4595.374, 3750.572)
    //      ECEF[m] ~ (-2.323892e6, -4.595374e6, 3.750572e6)

    Eigen::Vector3d point_llh(36.250278, -116.825833, -85.9536);
    Eigen::Vector3d point_ECEF(-2.323892e6, -4.595374e6, 3.750572e6);

    Eigen::Vector3d point_ECEF_Results = ecefPointFromLLH(point_llh);

    // Higher precision inputs should have tighter check
    ASSERT_NEAR(point_ECEF(0), point_ECEF_Results(0), 1e1);
    ASSERT_NEAR(point_ECEF(1), point_ECEF_Results(1), 1e1);
    ASSERT_NEAR(point_ECEF(2), point_ECEF_Results(2), 1e1);

    // ECEF back to LLH
    Eigen::Vector3d point_llh_results = llhPointFromECEF(point_ECEF_Results);

    ASSERT_NEAR(point_llh(0), point_llh_results(0), 1e-2);
    ASSERT_NEAR(point_llh(1), point_llh_results(1), 1e-2);
    ASSERT_NEAR(point_llh(2), point_llh_results(2), 1.0);
}

}  // namespace wave
