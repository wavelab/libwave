#include "wave/wave_test.hpp"
#include "wave/odometry/transformer.hpp"
#include "wave/matching/pointcloud_display.hpp"

namespace wave{

TEST(Transformer, Constructor) {
    TransformerParams params;
    Transformer transformer(params);
}

/// Just test no segfaults when interpolating transform
TEST(Transformer, update) {
    std::vector<PoseVel, Eigen::aligned_allocator<PoseVel>> trajectory;
    std::vector<float> stamps;

    TransformerParams params;
    params.n_scans = 4;
    params.traj_resolution = 3;
    uint32_t cnt = params.n_scans * (params.traj_resolution - 1) + 1;

    Vec6 acc;
    acc << 0, 0, 1.5, 3, 0, 0;

    stamps.resize(cnt);
    trajectory.resize(cnt);

    for(uint32_t i = 0; i < cnt; i++) {
        stamps.at(i) = static_cast<float>(i) * 0.1f;
        if (i != 0) {
            trajectory.at(i).pose = trajectory.at(i - 1).pose;
            trajectory.at(i).pose.manifoldPlus(trajectory.at(i-1).vel * 0.1);
        }
        trajectory.at(i).vel = static_cast<double>(i) * 0.1 * acc;
    }
    Transformer transformer(params);
    transformer.update(trajectory, stamps);
}

/// Each scan consists of 100 points sampled from a circle
TEST(Transformer, transformToStart) {
    std::vector<PoseVel, Eigen::aligned_allocator<PoseVel>> trajectory;
    std::vector<float> stamps;

    TransformerParams params;
    params.n_scans = 4;
    params.traj_resolution = 3;
    uint32_t cnt = params.n_scans * (params.traj_resolution - 1) + 1;

    Vec6 acc;
    acc << 0, 0, 1.5, 3, 0, 0;

    stamps.resize(cnt);
    trajectory.resize(cnt);

    for(uint32_t i = 0; i < cnt; i++) {
        stamps.at(i) = static_cast<float>(i) * 0.1f;
        if (i != 0) {
            trajectory.at(i).pose = trajectory.at(i - 1).pose;
            trajectory.at(i).pose.manifoldPlus(trajectory.at(i-1).vel * 0.1);
        }
        trajectory.at(i).vel = static_cast<double>(i) * 0.1 * acc;
    }
    Transformer transformer(params);
    transformer.update(trajectory, stamps);

    long n_pts = 100;
    Eigen::Tensor<float, 2> scan(4, n_pts);
    MatXf tscan;
    for (long i = 0; i < n_pts; i++) {
        scan(0,i) = static_cast<float>(std::cos(2*M_PI * static_cast<double>(i)/100.0));
        scan(1,i) = static_cast<float>(std::sin(2*M_PI * static_cast<double>(i)/100.0));
        scan(2,i) = 0.0f;
        scan(3,i) = 0.4f + 0.2f * (static_cast<float>(i)/100.0f);
    }
    EXPECT_NO_THROW(transformer.transformToStart(scan, tscan, 0));
}

/// Given a set of points in a circle, this should stretch/transform it smoothly across trajectory
/// Sanity check
TEST(Transformer, transformViz) {
    std::vector<PoseVel, Eigen::aligned_allocator<PoseVel>> trajectory;
    std::vector<float> stamps;

    TransformerParams params;
    params.n_scans = 4;
    params.traj_resolution = 3;
//    params.delRTol = 1e-3;
    uint32_t cnt = params.n_scans * (params.traj_resolution - 1) + 1;

    Vec6 vel;
    vel << 0, 0, 0, 3, 3, 0;

    stamps.resize(cnt);
    trajectory.resize(cnt);

    for(uint32_t i = 0; i < cnt; i++) {
        stamps.at(i) = static_cast<float>(i) * 0.1f;
        if (i != 0) {
//            trajectory.at(i).pose = trajectory.at(i - 1).pose;
//            trajectory.at(i).pose.manifoldPlus(vel * 0.1);
            trajectory.at(i).pose.setIdentity();
        }
        if (i % 2 == 0) {
            vel << 0, 0, 0, 3, 3, 0;
        } else {
            vel << 0, 0, 0, 3, -3, 0;
        }
        trajectory.at(i).vel = vel;
    }
    Transformer transformer(params);
    transformer.update(trajectory, stamps);

    long n_pts = 100;
    Eigen::Tensor<float, 2> scan(4, params.n_scans * n_pts);
    MatXf tscan, bscan;

    for (long j = 0; j < params.n_scans; j++) {
        for (long i = 0; i < n_pts; i++) {
            scan(0,j*n_pts + i) = static_cast<float>(2.0);
            scan(1,j*n_pts + i) = static_cast<float>(2.0);
            scan(2,j*n_pts + i) = 0.1f * (float)j + 0.1f * (static_cast<float>(i)/100.0f);
            scan(3,j*n_pts + i) = 0.2f * (static_cast<float>(i)/100.0f);
        }
    }

    transformer.transformToStart(scan, tscan, 0);
    transformer.transformToEnd(scan, bscan, 0);

    pcl::PointCloud<pcl::PointXYZI> transformed, original, btransformed;
    for (long i = 0; i < params.n_scans * n_pts; i++) {
        pcl::PointXYZI tpt, orpt, bpt;
        tpt.x = tscan(0,i);
        tpt.y = tscan(1,i);
        tpt.z = tscan(2,i);
        tpt.intensity = scan(3,i);

        bpt.x = bscan(0,i);
        bpt.y = bscan(1,i);
        bpt.z = bscan(2,i);
        bpt.intensity = scan(3,i);

        orpt.x = scan(0,i);
        orpt.y = scan(1,i);
        orpt.z = scan(2,i);
        orpt.intensity = scan(3,i);

        transformed.push_back(tpt);
        btransformed.push_back(bpt);
        original.push_back(orpt);
    }

    /// The "undistorted" scans should only be a rigid transform apart. Test with constant transform function
    // Transforming points in the frame at the start to the frame at the end
    MatXf btscan;

    transformer.constantTransform(0, params.n_scans - 1, tscan, btscan);

    MatXf error = btscan - bscan;

    EXPECT_NEAR(error.cwiseAbs().maxCoeff(), 0.0f, 1e-6);

    PointCloudDisplay display("warped");
    display.startSpin();
    display.addPointcloud(transformed.makeShared(), 0);
    display.addPointcloud(original.makeShared(), 1);
    display.addPointcloud(btransformed.makeShared(), 2);

    std::this_thread::sleep_for(std::chrono::seconds(10));
}

// This test generates synthetic data modelling a single-beam lidar moving in pure translation in a circular
// environment. Given the true transform, the transformer should perfectly undistort the transform
TEST(Transformer, CircleUndistort) {
    long n_pts = 100;
    Eigen::Tensor<float, 2> scan(4, n_pts);

    for (int i = 0; i < n_pts; ++i) {
        float tau = (float)(i) / (float)(n_pts);
        float px = tau;
        float nx = std::cos(tau * 2.0 * M_PI);
        float ny = std::sin(tau * 2.0 * M_PI);

        float t = -nx * px + std::sqrt(nx * nx * px * px - px * px + 4);

        scan(0,i) = t * nx;
        scan(1,i) = t * ny;
        scan(2,i) = 0;
        scan(3,i) = tau;
    }

    TransformerParams params;
    params.n_scans = 1;
    params.traj_resolution = 3;
    uint32_t cnt = params.n_scans * (params.traj_resolution - 1) + 1;

    Vec6 vel;
    vel << 0, 0, 0, 1, 0, 0;

    std::vector<PoseVel, Eigen::aligned_allocator<PoseVel>> trajectory;
    std::vector<float> stamps;

    stamps.resize(cnt);
    trajectory.resize(cnt);

    for(uint32_t i = 0; i < cnt; i++) {
        stamps.at(i) = static_cast<float>(i) * 0.5f;
        if (i != 0) {
            trajectory.at(i).pose = trajectory.at(i - 1).pose;
            trajectory.at(i).pose.manifoldPlus(vel * 0.5);
        } else {
            trajectory.at(i).pose.setIdentity();
        }
        trajectory.at(i).vel = vel;
    }
    Transformer transformer(params);
    transformer.update(trajectory, stamps);

    MatXf bscan, escan;

    transformer.transformToStart(scan, bscan, 0);
    transformer.transformToEnd(scan, escan, 0);

    pcl::PointCloud<pcl::PointXYZI> beginning, original, ending;
    for (long i = 0; i < params.n_scans * n_pts; i++) {
        pcl::PointXYZI tpt, orpt, bpt;
        tpt.x = escan(0,i);
        tpt.y = escan(1,i);
        tpt.z = escan(2,i);
        tpt.intensity = scan(3,i);

        bpt.x = bscan(0,i);
        bpt.y = bscan(1,i);
        bpt.z = bscan(2,i);
        bpt.intensity = scan(3,i);

        orpt.x = scan(0,i);
        orpt.y = scan(1,i);
        orpt.z = scan(2,i);
        orpt.intensity = scan(3,i);

        beginning.push_back(bpt);
        original.push_back(orpt);
        ending.push_back(tpt);

        EXPECT_NEAR(bscan(0,i)*bscan(0,i) + bscan(1,i) * bscan(1,i), 4.0, 1e-3);
    }

    PointCloudDisplay display("warped");
    display.startSpin();
    display.addPointcloud(original.makeShared(), 0);
    display.addPointcloud(beginning.makeShared(), 1);
    display.addPointcloud(ending.makeShared(), 2);

    std::this_thread::sleep_for(std::chrono::seconds(10));
}

}
