#include "wave/wave_test.hpp"
#include "wave/odometry/transformer.hpp"
#include "wave/matching/pointcloud_display.hpp"

namespace wave{

TEST(Transformer, Transformer) {
    TransformerParams params;
    Transformer transformer(params);
}

/// Just test no segfaults when interpolating transform
TEST(Transformer, update) {
    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> trajectory;
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
    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> trajectory;
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
    Eigen::Tensor<float, 2> tscan(4, n_pts);
    for (long i = 0; i < n_pts; i++) {
        scan(0,i) = static_cast<float>(std::cos(2*M_PI * static_cast<double>(i)/100.0));
        scan(1,i) = static_cast<float>(std::sin(2*M_PI * static_cast<double>(i)/100.0));
        scan(2,i) = 0.0f;
        scan(3,i) = 0.4f + 0.2f * (static_cast<float>(i)/100.0f);
    }
    EXPECT_NO_THROW(transformer.transformToStart(scan, tscan, <#initializer#>));
}

/// Given a set of points in a circle, this should stretch/transform it smoothly across trajectory
/// Sanity check
TEST(Transformer, transformViz) {
    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> trajectory;
    std::vector<float> stamps;

    TransformerParams params;
    params.n_scans = 4;
    params.traj_resolution = 3;
    uint32_t cnt = params.n_scans * (params.traj_resolution - 1) + 1;

    Vec6 vel;
    vel << 0, 0, 1.5, 3, 0, 2;

    stamps.resize(cnt);
    trajectory.resize(cnt);

    for(uint32_t i = 0; i < cnt; i++) {
        stamps.at(i) = static_cast<float>(i) * 0.1f;
        if (i != 0) {
            trajectory.at(i).pose = trajectory.at(i - 1).pose;
            trajectory.at(i).pose.manifoldPlus(vel * 0.1);
        }
        trajectory.at(i).vel = vel;
    }
    Transformer transformer(params);
    transformer.update(trajectory, stamps);

    long n_pts = 100;
    Eigen::Tensor<float, 2> scan(4, params.n_scans * n_pts);
    Eigen::Tensor<float, 2> tscan(4, params.n_scans * n_pts);

    for (long j = 0; j < params.n_scans; j++) {
        for (long i = 0; i < n_pts; i++) {
            scan(0,j*n_pts + i) = static_cast<float>(std::cos(2*M_PI * static_cast<double>(i)/100.0));
            scan(1,j*n_pts + i) = static_cast<float>(std::sin(2*M_PI * static_cast<double>(i)/100.0));
            scan(2,j*n_pts + i) = 0.0f;
            scan(3,j*n_pts + i) = 0.2f * (float)j + 0.2f * (static_cast<float>(i)/100.0f);
        }
    }

    transformer.transformToStart(scan, tscan, <#initializer#>);

    pcl::PointCloud<pcl::PointXYZI> transformed, original;
    for (long i = 0; i < params.n_scans * n_pts; i++) {
        pcl::PointXYZI tpt, orpt;
        tpt.x = tscan(0,i);
        tpt.y = tscan(1,i);
        tpt.z = tscan(2,i);
        tpt.intensity = scan(3,i);

        orpt.x = scan(0,i);
        orpt.y = scan(1,i);
        orpt.z = scan(2,i);
        orpt.intensity = scan(3,i);

        transformed.push_back(tpt);
        original.push_back(orpt);
    }

    PointCloudDisplay display("warped");
    display.startSpin();
    display.addPointcloud(transformed.makeShared(), 0);
    display.addPointcloud(original.makeShared(), 1);

    cin.get();
}

}
