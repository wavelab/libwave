#include <gtest/gtest.h>

#include "wave/kinematics/gimbal.hpp"

namespace wave {

#define ATTITUDE_CONTROLLER_OUTPUT "/tmp/gimbal_attitude_controller.dat"

static void print_target_bf(Vec3 &target) {
    std::cout << "target position (relative to quad in body frame): ";
    std::cout << std::fixed << std::setprecision(2) << target(0) << " ";
    std::cout << std::fixed << std::setprecision(2) << target(1) << " ";
    std::cout << std::fixed << std::setprecision(2) << target(2) << std::endl;
}

static void print_target_bpf(std::string prefix, Vec3 &target) {
    std::cout << prefix;
    std::cout << "target position (relative to quad in body planar frame): ";
    std::cout << std::fixed << std::setprecision(2) << target(0) << " ";
    std::cout << std::fixed << std::setprecision(2) << target(1) << " ";
    std::cout << std::fixed << std::setprecision(2) << target(2) << std::endl;
}

TEST(GimbalGimbal2AxisController, constructor) {
    Gimbal2AxisController controller;
}

TEST(GimbalGimbal2AxisController, update) {
    double t, dt;
    FILE *output_file;
    Gimbal2AxisModel gimbal;
    Gimbal2AxisController controller;
    Vec2 setpoints;
    Vec2 actual;
    Vec2 motor_inputs;

    // setup
    setpoints << deg2rad(10.0), deg2rad(-10.0);
    output_file = fopen(ATTITUDE_CONTROLLER_OUTPUT, "w");
    fprintf(output_file, "t, roll, pitch\n");

    // control attitude
    t = 0;
    dt = 0.001;
    for (int i = 0; i < 1000; i++) {
        actual << gimbal.states(0), gimbal.states(2);
        motor_inputs = controller.update(setpoints, actual, dt);
        gimbal.update(motor_inputs, dt);

        fprintf(output_file, "%f, ", t);
        fprintf(output_file, "%f, ", rad2deg(gimbal.states(0)));
        fprintf(output_file, "%f\n", rad2deg(gimbal.states(2)));
        t += dt;
    }

    // clean up
    fclose(output_file);
}

TEST(Gimbal2AxisModel, getTargetInBF) {
    Gimbal2AxisModel gimbal;
    Vec3 target_cf;
    Vec3 target_bf;

    double roll = 0.0;
    double pitch = deg2rad(90);
    double yaw = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;

    gimbal.camera_offset = Pose(roll, pitch, yaw, dx, dy, dz);

    // target is front of camera
    target_cf << 0.0, 0.0, 1.0;
    target_bf = gimbal.getTargetInBF(target_cf);
    std::cout << "[target is front of camera]\t\t";
    print_target_bf(target_bf);
    EXPECT_NEAR(0.0, target_bf(0), 0.0000001);
    EXPECT_NEAR(0.0, target_bf(1), 0.0000001);
    EXPECT_NEAR(-1.0, target_bf(2), 0.0000001);

    // target left of camera
    target_cf << -1.0, 0.0, 0.0;
    target_bf = gimbal.getTargetInBF(target_cf);
    std::cout << "[target is left of camera]\t\t";
    print_target_bf(target_bf);
    EXPECT_NEAR(0.0, target_bf(0), 0.0000001);
    EXPECT_NEAR(1.0, target_bf(1), 0.0000001);
    EXPECT_NEAR(0.0, target_bf(2), 0.0000001);

    // target is right of camera
    target_cf << 1.0, 0.0, 0.0;
    target_bf = gimbal.getTargetInBF(target_cf);
    std::cout << "[target is right of camera]\t\t";
    print_target_bf(target_bf);
    EXPECT_NEAR(0.0, target_bf(0), 0.0000001);
    EXPECT_NEAR(-1.0, target_bf(1), 0.0000001);
    EXPECT_NEAR(0.0, target_bf(2), 0.0000001);

    // target is top of camera
    target_cf << 0.0, -1.0, 0.0;
    target_bf = gimbal.getTargetInBF(target_cf);
    std::cout << "[target is top of camera]\t\t";
    print_target_bf(target_bf);
    EXPECT_NEAR(1.0, target_bf(0), 0.0000001);
    EXPECT_NEAR(0.0, target_bf(1), 0.0000001);
    EXPECT_NEAR(0.0, target_bf(2), 0.0000001);

    // target is bottom of camera
    target_cf << 0.0, 1.0, 0.0;
    target_bf = gimbal.getTargetInBF(target_cf);
    std::cout << "[target is bottom of camera]\t\t";
    print_target_bf(target_bf);
    EXPECT_NEAR(-1.0, target_bf(0), 0.0000001);
    EXPECT_NEAR(0.0, target_bf(1), 0.0000001);
    EXPECT_NEAR(0.0, target_bf(2), 0.0000001);

    // target is top-left of camera
    target_cf << -1.0, -1.0, 0.0;
    target_bf = gimbal.getTargetInBF(target_cf);
    std::cout << "[target is top-left of camera]\t\t";
    print_target_bf(target_bf);
    EXPECT_NEAR(1.0, target_bf(0), 0.0000001);
    EXPECT_NEAR(1.0, target_bf(1), 0.0000001);
    EXPECT_NEAR(0.0, target_bf(2), 0.0000001);

    // target is top-right of camera
    target_cf << 1.0, -1.0, 0.0;
    target_bf = gimbal.getTargetInBF(target_cf);
    std::cout << "[target is top-right of camera]\t\t";
    print_target_bf(target_bf);
    EXPECT_NEAR(1.0, target_bf(0), 0.0000001);
    EXPECT_NEAR(-1.0, target_bf(1), 0.0000001);
    EXPECT_NEAR(0.0, target_bf(2), 0.0000001);

    // target is bottom-left of camera
    target_cf << -1.0, 1.0, 0.0;
    target_bf = gimbal.getTargetInBF(target_cf);
    std::cout << "[target is bottom-left of camera]\t";
    print_target_bf(target_bf);
    EXPECT_NEAR(-1.0, target_bf(0), 0.0000001);
    EXPECT_NEAR(1.0, target_bf(1), 0.0000001);
    EXPECT_NEAR(0.0, target_bf(2), 0.0000001);

    // target is bottom-right of camera
    target_cf << 1.0, 1.0, 0.0;
    target_bf = gimbal.getTargetInBF(target_cf);
    std::cout << "[target is bottom-right of camera]\t";
    print_target_bf(target_bf);
    EXPECT_NEAR(-1.0, target_bf(0), 0.0000001);
    EXPECT_NEAR(-1.0, target_bf(1), 0.0000001);
    EXPECT_NEAR(0.0, target_bf(2), 0.0000001);
}

TEST(Gimbal2AxisModel, getTargetInBPF) {
    Gimbal2AxisModel gimbal;
    Vec3 euler, target_cf, target_bpf;
    Quaternion frame, joints;
    double roll, pitch, yaw;
    double dx, dy, dz;

    // setup
    roll = 0.0;
    pitch = deg2rad(90);
    yaw = 0.0;

    dx = 0.0;
    dy = 0.0;
    dz = 0.0;

    target_cf << 0.0, 0.0, 10.0;  // let tag be directly infront of camera
    gimbal.camera_offset = Pose(roll, pitch, yaw, dx, dy, dz);
    std::cout << "target is directly infront of camera: ";
    std::cout << "[" << target_cf.transpose() << "]" << std::endl;

    // gimbal point backwards
    euler << 0.0, deg2rad(10), 0.0;
    euler2quat(euler, 321, joints);

    euler << 0.0, 0.0, 0.0;
    euler2quat(euler, 321, frame);

    target_bpf = gimbal.getTargetInBPF(target_cf, frame, joints);
    print_target_bpf("[gimbal point backwards]\t", target_bpf);

    EXPECT_TRUE(target_bpf(0) < 0.0);
    EXPECT_NEAR(0.0, target_bpf(1), 0.000000001);
    EXPECT_TRUE(target_bpf(2) < 0.0);

    // gimbal point forwards
    euler << 0.0, deg2rad(-10), 0.0;
    euler2quat(euler, 321, joints);

    euler << 0.0, 0.0, 0.0;
    euler2quat(euler, 321, frame);

    target_bpf = gimbal.getTargetInBPF(target_cf, frame, joints);
    print_target_bpf("[gimbal point forwards]\t\t", target_bpf);

    EXPECT_TRUE(target_bpf(0) > 0.0);
    EXPECT_NEAR(0.0, target_bpf(1), 0.000000001);
    EXPECT_TRUE(target_bpf(2) < 0.0);

    // gimbal point right
    euler << deg2rad(-10), 0.0, 0.0;
    euler2quat(euler, 321, joints);

    euler << 0.0, 0.0, 0.0;
    euler2quat(euler, 321, frame);

    target_bpf = gimbal.getTargetInBPF(target_cf, frame, joints);
    print_target_bpf("[gimbal point right]\t\t", target_bpf);

    EXPECT_NEAR(0.0, target_bpf(0), 0.0000000001);
    EXPECT_TRUE(target_bpf(1) < 0.0);
    EXPECT_TRUE(target_bpf(2) < 0.0);

    // gimbal point left
    euler << deg2rad(10), 0.0, 0.0;
    euler2quat(euler, 321, joints);

    euler << deg2rad(0), 0.0, 0.0;
    euler2quat(euler, 321, frame);

    target_bpf = gimbal.getTargetInBPF(target_cf, frame, joints);
    print_target_bpf("[gimbal point left]\t\t", target_bpf);

    EXPECT_NEAR(0.0, target_bpf(0), 0.0000000001);
    EXPECT_TRUE(target_bpf(1) > 0.0);
    EXPECT_TRUE(target_bpf(2) < 0.0);

    // gimbal point backwards right
    euler << deg2rad(-10), deg2rad(10), 0.0;
    euler2quat(euler, 321, joints);

    euler << deg2rad(0), 0.0, 0.0;
    euler2quat(euler, 321, frame);

    target_bpf = gimbal.getTargetInBPF(target_cf, frame, joints);
    print_target_bpf("[gimbal point backwards right]\t", target_bpf);

    EXPECT_TRUE(target_bpf(0) < 0.0);
    EXPECT_TRUE(target_bpf(1) < 0.0);
    EXPECT_TRUE(target_bpf(2) < 0.0);

    // gimbal point backwards left
    euler << deg2rad(10), deg2rad(10), 0.0;
    euler2quat(euler, 321, joints);

    euler << deg2rad(0), 0.0, 0.0;
    euler2quat(euler, 321, frame);

    target_bpf = gimbal.getTargetInBPF(target_cf, frame, joints);
    print_target_bpf("[gimbal point backwards left]\t", target_bpf);

    EXPECT_TRUE(target_bpf(0) < 0.0);
    EXPECT_TRUE(target_bpf(1) > 0.0);
    EXPECT_TRUE(target_bpf(2) < 0.0);

    // gimbal point forwards right
    euler << deg2rad(-10), deg2rad(-10), 0.0;
    euler2quat(euler, 321, joints);

    euler << deg2rad(0), 0.0, 0.0;
    euler2quat(euler, 321, frame);

    target_bpf = gimbal.getTargetInBPF(target_cf, frame, joints);
    print_target_bpf("[gimbal point forwards right]\t", target_bpf);

    EXPECT_TRUE(target_bpf(0) > 0.0);
    EXPECT_TRUE(target_bpf(1) < 0.0);
    EXPECT_TRUE(target_bpf(2) < 0.0);

    // gimbal point forwards left
    euler << deg2rad(10), deg2rad(-10), 0.0;
    euler2quat(euler, 321, joints);

    euler << deg2rad(0), 0.0, 0.0;
    euler2quat(euler, 321, frame);

    target_bpf = gimbal.getTargetInBPF(target_cf, frame, joints);
    print_target_bpf("[gimbal point forwards left]\t", target_bpf);

    EXPECT_TRUE(target_bpf(0) > 0.0);
    EXPECT_TRUE(target_bpf(1) > 0.0);
    EXPECT_TRUE(target_bpf(2) < 0.0);
}

TEST(Gimbal2AxisModel, getTargetInBPF2) {
    Gimbal2AxisModel gimbal;
    Vec3 euler, target_cf, target_bpf;
    Quaternion frame, joints;
    double roll, pitch, yaw;
    double dx, dy, dz;

    // setup
    roll = 0.0;
    pitch = deg2rad(90);
    yaw = 0.0;

    dx = 0.0;
    dy = 0.0;
    dz = 0.0;

    target_cf << 0.0, 0.0, 10.0;  // let tag be directly infront of camera
    gimbal.camera_offset = Pose(roll, pitch, yaw, dx, dy, dz);
    std::cout << "target is directly infront of camera: ";
    std::cout << "[" << target_cf.transpose() << "]" << std::endl;

    // gimbal point backwards, body pitch backwards
    euler << 0.0, deg2rad(10), 0.0;
    euler2quat(euler, 321, joints);

    euler << 0.0, deg2rad(-10), 0.0;
    euler2quat(euler, 321, frame);

    target_bpf = gimbal.getTargetInBPF(target_cf, frame, joints);
    print_target_bpf("[gimbal point backwards, body pitch backwards]\t\t",
                     target_bpf);

    EXPECT_NEAR(0.0, target_bpf(0), 0.01);
    EXPECT_NEAR(0.0, target_bpf(1), 0.01);
    EXPECT_NEAR(-10.0, target_bpf(2), 0.01);

    // gimbal point left, body roll left
    euler << deg2rad(10), 0.0, 0.0;
    euler2quat(euler, 321, joints);

    euler << deg2rad(-10), 0.0, 0.0;
    euler2quat(euler, 321, frame);

    target_bpf = gimbal.getTargetInBPF(target_cf, frame, joints);
    print_target_bpf("[gimbal point left, body roll left]\t\t\t", target_bpf);

    EXPECT_NEAR(0.0, target_bpf(0), 0.01);
    EXPECT_NEAR(0.0, target_bpf(1), 0.01);
    EXPECT_NEAR(-10.0, target_bpf(2), 0.01);
}

TEST(Gimbal2AxisModel, trackTarget) {
    Gimbal2AxisModel gimbal;
    Vec3 euler, target_cf, target_bpf;
    double roll, pitch, yaw;
    double dx, dy, dz;

    // setup
    roll = 0.0;
    pitch = deg2rad(90);
    yaw = 0.0;
    dx = 0.0;
    dy = 0.0;
    dz = 0.0;

    target_cf << 0.0, 0.0, 3.0;  // let tag be directly infront of camera
    gimbal.camera_offset = Pose(roll, pitch, yaw, dx, dy, dz);
    std::cout << "target is directly infront of camera: ";
    std::cout << "[" << target_cf.transpose() << "]" << std::endl;

    // quadrotor hovering
    gimbal.states(0) = 0.0;
    gimbal.states(2) = deg2rad(0.0);
    gimbal.trackTarget(target_cf);
    std::cout << "[quadrotor hovering]\t\t";
    std::cout << gimbal.joint_setpoints.transpose() << std::endl;

    // quadrotor pitched forwards
    gimbal.states(0) = 0.0;
    gimbal.states(2) = deg2rad(10.0);
    gimbal.trackTarget(target_cf);
    std::cout << "[quadrotor pitched forwards]\t\t";
    std::cout << gimbal.joint_setpoints.transpose() << std::endl;
}

}  // end of wave namespace
