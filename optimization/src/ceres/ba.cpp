#include "wavelib/optimization/ceres/ba.hpp"


namespace wavelib {
namespace ceres {

BundleAdjustment::BundleAdjustment(void)
{
    this->configured = false;

    this->K = MatX::Zero(3, 3);
    this->x1_pts.resize(0, 0);
    this->x2_pts.resize(0, 0);

    this->q = NULL;
    this->c = NULL;
    this->x = NULL;
}

BundleAdjustment::~BundleAdjustment(void)
{
    if (this->configured) {
        for (int i = 0; i < this->x1_pts.rows(); i++) {
            free(this->x[i]);
        }
        free(this->q[0]);
        free(this->q[1]);

        free(this->c[0]);
        free(this->c[1]);

        free(this->x);
        free(this->q);
        free(this->c);
    }
}

int BundleAdjustment::configure(Mat3 K, MatX x1_pts, MatX x2_pts)
{
    this->configured = true;

    this->K = K;
    this->x1_pts = x1_pts;
    this->x2_pts = x2_pts;

    this->q = (double **) malloc(sizeof(double *) * 2);
    this->c = (double **) malloc(sizeof(double *) * 2);
    this->x = (double **) malloc(sizeof(double *) * this->x1_pts.rows());

    // initialize quaternion and camera center position
    for (int i = 0; i < 2; i++) {
        // quaternion q = (x, y, z, w)
        this->q[i] = (double *) malloc(sizeof(double) * 4);
        this->q[i][0] = 0.0;
        this->q[i][1] = 0.0;
        this->q[i][2] = 0.0;
        this->q[i][3] = 1.0;

        // camera center c = (x, y, z)
        this->c[i] = (double *) malloc(sizeof(double) * 3);
        this->c[i][0] = 0.0;
        this->c[i][1] = 0.0;
        this->c[i][2] = 0.0;
    }

    // initialize 3D points (x, y, z)
    Vec3 pt;
    Vec3 x1;
    Mat3 K_inv;

    K_inv = this->K.inverse();
    for (int i = 0; i < this->x1_pts.rows(); i++) {
        x1 << this->x1_pts(i, 0), this->x1_pts(i, 1), 1.0;
        pt =  K_inv * x1;

        this->x[i] = (double *) malloc(sizeof(double) * 3);
        this->x[i][0] = pt(0);
        this->x[i][1] = pt(1);
        this->x[i][2] = 1.0;
    }

    return 0;
}

int BundleAdjustment::solve(MatX pts3d)
{
    Vec2 pt1, pt2;
    ::ceres::Problem problem;
    ::ceres::Solver::Options options;
    ::ceres::Solver::Summary summary;
    ::ceres::AutoDiffCostFunction<BAResidual, 2, 4, 3, 3> *cost_func;
    BAResidual *r;

    // options
    options.max_num_iterations = 200;
    options.use_nonmonotonic_steps = false;
    options.use_inner_iterations = true;
    options.preconditioner_type = ::ceres::SCHUR_JACOBI;
    options.linear_solver_type = ::ceres::SPARSE_SCHUR;
    options.parameter_tolerance = 1e-10;
    options.num_threads = 1;
    options.num_linear_solver_threads = 1;
    // options.minimizer_progress_to_stdout = true;

    ::ceres::LocalParameterization *quat_param;
    quat_param = new ceres::extensions::EigenQuaternionParameterization();

    // image 1
    for (int i = 0; i < this->x1_pts.rows(); i++) {
        pt1 << this->x1_pts(i, 0), this->x1_pts(i, 1);
        r = new BAResidual(this->K, pt1, true);

        cost_func = new ::ceres::AutoDiffCostFunction<
            BAResidual,
            2, // size of residual
            4, // size of 1st parameter - quaternion
            3, // size of 2nd parameter - camera center (x, y, z)
            3  // size of 3rd parameter - 3d point in world (x, y, z)
        >(r);
        problem.AddResidualBlock(
            cost_func,
            NULL,
            this->q[0],
            this->c[0],
            this->x[i]
        );
    }
    problem.SetParameterization(q[0], quat_param);

    // image 2
    for (int i = 0; i < this->x1_pts.rows(); i++) {
        pt2 << this->x2_pts(i, 0), this->x2_pts(i, 1);
        r = new BAResidual(this->K, pt2, false);

        cost_func = new ::ceres::AutoDiffCostFunction<
            BAResidual,
            2, // size of residual
            4, // size of 1st parameter - quaternion
            3, // size of 2nd parameter - camera center (x, y, z)
            3  // size of 3rd parameter - 3d point in world (x, y, z)
        >(r);
        problem.AddResidualBlock(
            cost_func,
            NULL,
            this->q[1],
            this->c[1],
            this->x[i]
        );
    }
    problem.SetParameterization(q[1], quat_param);

    // solve
    ::ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    return 0;
}

}  // end of ceres namespace
}  // end of wavelib namespace
