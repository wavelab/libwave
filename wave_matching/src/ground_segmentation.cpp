#include <Eigen/Eigenvalues>
#include <wave/matching/ground_segmentation.hpp>
#include <wave/matching/PointcloudXYZGD.hpp>
#include <wave/utils/log.hpp>

namespace wave {

bool compareSignalPoints(const SignalPoint &a, const SignalPoint &b) {
    return a.height < b.height;
}

GroundSegmentation::GroundSegmentation(GroundSegmentationParams config) {
    this->params = config;
    this->polar_bin_grid = new PolarBinGrid;
    initializePolarBinGrid();
}

void GroundSegmentation::initializePolarBinGrid(void) {
    this->polar_bin_grid->ang_cell.resize(this->params.num_bins_a);
    for (int i = 0; i < this->params.num_bins_a; i++) {
        this->polar_bin_grid->ang_cell[i].sig_points.clear();
        this->polar_bin_grid->ang_cell[i].lin_cell.resize(
          this->params.num_bins_l);
        this->polar_bin_grid->ang_cell[i].sig_points.resize(
          this->params.num_bins_l);
        this->polar_bin_grid->ang_cell[i].range_height_signal.resize(
          this->params.num_bins_l);
        std::vector<SignalPoint>().swap(polar_bin_grid->ang_cell[i].sig_points);
        for (int j = 0; j < this->params.num_bins_l; j++) {
            polar_bin_grid->ang_cell[i].lin_cell[j].bin_points.clear();
            polar_bin_grid->ang_cell[i].lin_cell[j].obs_points.clear();
            polar_bin_grid->ang_cell[i].lin_cell[j].drv_points.clear();
            polar_bin_grid->ang_cell[i].lin_cell[j].ground_points.clear();
            polar_bin_grid->ang_cell[i].lin_cell[j].prototype_point =
              PointXYZGD();
            polar_bin_grid->ang_cell[i].lin_cell[j].prototype_point.x = NAN;
            polar_bin_grid->ang_cell[i].lin_cell[j].prototype_point.y = NAN;
            polar_bin_grid->ang_cell[i].lin_cell[j].prototype_point.z = NAN;
            polar_bin_grid->ang_cell[i].range_height_signal[j].x = NAN;
            polar_bin_grid->ang_cell[i].range_height_signal[j].y = NAN;
            polar_bin_grid->ang_cell[i].lin_cell[j].cluster_assigned = -1;

            // force memory deletion of std::vectors
            std::vector<PointXYZGD>().swap(
              polar_bin_grid->ang_cell[i].lin_cell[j].bin_points);
            std::vector<PointXYZGD>().swap(
              polar_bin_grid->ang_cell[i].lin_cell[j].obs_points);
            std::vector<PointXYZGD>().swap(
              polar_bin_grid->ang_cell[i].lin_cell[j].drv_points);
            std::vector<PointXYZGD>().swap(
              polar_bin_grid->ang_cell[i].lin_cell[j].ground_points);
        }
    }
}

void GroundSegmentation::setupGroundSegmentation(
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
  pcl::PointCloud<PointXYZGD>::Ptr out_ground_cloud,
  pcl::PointCloud<PointXYZGD>::Ptr out_obs_cloud,
  pcl::PointCloud<PointXYZGD>::Ptr out_drv_cloud) {
    ref_cloud = input_cloud;  // set the cloud datastructure;
    // Save the given pointers, and clear the output clouds
    this->obs_cloud = out_obs_cloud;
    this->ground_cloud = out_ground_cloud;
    this->drv_cloud = out_drv_cloud;
    this->obs_cloud->clear();
    this->ground_cloud->clear();
    this->drv_cloud->clear();

    this->genPolarBinGrid(ref_cloud);
}

void GroundSegmentation::genPolarBinGrid(
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    initializePolarBinGrid();

    size_t num_points = input_cloud->size();
    double bsize_rad = (double) ((360.0) / this->params.num_bins_a);
    double bsize_lin = (double) this->params.rmax / this->params.num_bins_l;
    for (size_t i = 0; i < num_points; i++) {
        PointXYZGD cur_point;
        float px = cur_point.x = input_cloud->points[i].x;
        float py = cur_point.y = input_cloud->points[i].y;
        float pz = cur_point.z = input_cloud->points[i].z;

        if (sqrt(px * px + py * py + pz * pz) < this->params.rmax) {
            double ph = (atan2(py, px)) * (180 / M_PI);  // in degrees
            if (ph < 0)
                ph = 360.0 + ph;

            // bin into sector

            unsigned int bind_rad =
              static_cast<unsigned int>(ph / bsize_rad);  // got the radial bin

            // get the linear bin
            float xy_dist = std::sqrt(px * px + py * py);

            unsigned int bind_lin = static_cast<unsigned int>(
              xy_dist / bsize_lin);  // got the radial bin

            this->polar_bin_grid->ang_cell[bind_rad]
              .lin_cell[bind_lin]
              .bin_points.push_back(cur_point);
            // add the point to the bin
            // check the protoype point

            if (isnanf(polar_bin_grid->ang_cell[bind_rad]
                         .lin_cell[bind_lin]
                         .prototype_point.z) ||
                pz < polar_bin_grid->ang_cell[bind_rad]
                       .lin_cell[bind_lin]
                       .prototype_point.z)  // smallest by z
            {
                this->polar_bin_grid->ang_cell[bind_rad]
                  .lin_cell[bind_lin]
                  .prototype_point = cur_point;
                this->polar_bin_grid->ang_cell[bind_rad]
                  .range_height_signal[bind_lin]
                  .x = xy_dist;
                this->polar_bin_grid->ang_cell[bind_rad]
                  .range_height_signal[bind_lin]
                  .y = pz;
            }
        }
    }
}


MatX GroundSegmentation::genGPModel(std::vector<SignalPoint> &ps1,
                                    std::vector<SignalPoint> &ps2,
                                    float sig_f,
                                    float p_l) {
    size_t nP1 = ps1.size();
    size_t nP2 = ps2.size();

    MatX cmat;
    cmat.resize(nP1, nP2);
    float coeff = (-1 / (2 * p_l * p_l));

    for (size_t i = 0; i < nP1; i++) {
        for (size_t j = 0; j < nP2; j++) {
            double diff = (ps1[i].range - ps2[j].range);
            cmat(i, j) = sig_f * exp(coeff * (diff * diff));
        }
    }
    return cmat;
}

void GroundSegmentation::segmentGround() {
    for (int i = 0; i < this->params.num_bins_a; i++) {
        this->sectorINSAC(i);
    }
}

void GroundSegmentation::sectorINSAC(int sector_index) {
    if (sector_index >= this->params.num_bins_a) {
        return;
    }
    int num_filled = 0;

    // pull out the valid points from the sector
    auto &sig_points = polar_bin_grid->ang_cell[sector_index].sig_points;
    sig_points.clear();
    for (int i = 0; i < this->params.num_bins_l; i++) {
        if (!std::isnan(polar_bin_grid->ang_cell[sector_index]
                          .range_height_signal[i]
                          .x) &&
            this->polar_bin_grid->ang_cell[sector_index]
                .lin_cell[i]
                .bin_points.size() > 5) {
            // bin has a valid point, and enough points to make a good
            // guess for a protopoint
            SignalPoint new_point;
            new_point.range =
              polar_bin_grid->ang_cell[sector_index].range_height_signal[i].x;
            new_point.height =
              polar_bin_grid->ang_cell[sector_index].range_height_signal[i].y;
            new_point.index = i;
            sig_points.push_back(new_point);
            num_filled++;
        }
    }
    // get the seed points.  Select the 3 lowest points.  Sort based on height
    // values
    sort(sig_points.begin(), sig_points.end(), compareSignalPoints);

    // now that the z points are sorted by height, take the
    // this->params.num_seed_points worth
    // as the seed
    size_t num_points =
      sig_points.size() < static_cast<size_t>(this->params.num_seed_points)
        ? sig_points.size()
        : static_cast<size_t>(this->params.num_seed_points);
    std::vector<SignalPoint> current_model;
    int point_count = 0;
    int curr_idx = 0;
    bool keep_going = true;
    bool sufficient_model = true;

    while (true) {
        if (static_cast<size_t>(curr_idx) >= sig_points.size())  // overflow
        {
            break;
        }

        if (sig_points[curr_idx].range < this->params.max_seed_range &&
            fabs(sig_points[curr_idx].height) < this->params.max_seed_height) {
            // close enough to
            // robot and height
            // makese sense in
            // robot locality

            sig_points[curr_idx].is_ground = true;
            current_model.push_back(sig_points[curr_idx]);
            sig_points.erase(sig_points.begin() + curr_idx);
            point_count++;

        } else {
            curr_idx++;
        }

        if (static_cast<size_t>(point_count) >= num_points)  // done
        {
            break;
        }
    }

    // check size
    if (current_model.size() < 2)  // not enough for model, all obs pts
    {
        keep_going = false;
        sufficient_model = false;
    }

    // got the seedpoints, start theINSAC process
    // cov matrices
    MatX C_XsX;
    MatX C_XX;
    MatX C_XsXs;
    MatX C_XXs;

    if (sig_points.size() == 0)
        // no points to insac, put the seed points in as ground
        keep_going = false;

    MatX temp;
    MatX f_s;
    MatX Vf_s;
    while (keep_going) {
        // generate the covariance matrices

        C_XsX = genGPModel(
          sig_points, current_model, this->params.p_sf, this->params.p_l);
        C_XX = genGPModel(
          current_model, current_model, this->params.p_sf, this->params.p_l);
        C_XsXs = genGPModel(
          sig_points, sig_points, this->params.p_sf, this->params.p_l);
        C_XXs = C_XsX.transpose();

        // temporary calc
        MatX temp_calc1 =
          C_XX + (this->params.p_sn * MatX::Identity(C_XX.rows(), C_XX.cols()));
        MatX temp_calc2 = C_XsX * temp_calc1.inverse();

        // test the points against the current model

        MatX model_z(current_model.size(), 1);
        for (unsigned int i = 0; i < current_model.size(); i++) {
            model_z(i, 0) = current_model[i].height;
        }

        f_s = temp_calc2 * model_z;
        Vf_s = C_XsXs - temp_calc2 * C_XXs;

        if (Vf_s.rows() == 0) {
            keep_going = false;
            LOG_INFO("WARNING BREAKING LOOP: VF_s does not exist");
            continue;
        }

        bool search_candidate_points = true;
        unsigned int k = 0;
        // test for inliers using INSAC algorithm
        const auto start_size =
          current_model.size();  // beginning size of the model set
        while (search_candidate_points) {
            double vf = Vf_s(k, k);
            double met = (sig_points[k].height - f_s(k)) /
                         (sqrt(this->params.p_sn + vf * vf));

            if (vf < this->params.p_tmodel &&
                std::abs(met) < this->params.p_tdata) {  // we have an inlier!
                // add to model set
                current_model.push_back(sig_points[k]);
                // remove from sample set
                sig_points.erase(sig_points.begin() + k);

                // delete row from f_s
                temp = f_s;
                f_s.resize(f_s.rows() - 1, f_s.cols());
                f_s.topRows(k) = temp.topRows(k);
                f_s.bottomRows(temp.rows() - k - 1) =
                  temp.bottomRows(temp.rows() - k - 1);

                // delete row from Vf_s
                temp = Vf_s;
                Vf_s.resize(Vf_s.rows() - 1, Vf_s.cols());
                Vf_s.topRows(k) = temp.topRows(k);
                Vf_s.bottomRows(temp.rows() - k - 1) =
                  temp.bottomRows(temp.rows() - k - 1);

                // delete col from Vf_s
                temp = Vf_s;
                Vf_s.resize(Vf_s.rows(), Vf_s.cols() - 1);
                Vf_s.leftCols(k) = temp.leftCols(k);
                Vf_s.rightCols(temp.cols() - k - 1) =
                  temp.rightCols(temp.cols() - k - 1);

            } else {
                k++;
            }

            if (sig_points.size() == k) {
                search_candidate_points = false;
            }
        }

        const auto end_size =
          current_model.size();  // end size of the model set
        if (start_size == end_size || sig_points.size() == 0) {
            keep_going = false;
        }
    }  // end INSAC

    // fill in the ground and obs pointclouds

    double num_obs = 0;
    Vec3 obs_sum(0, 0, 0);

    for (int i = 0; i < (int) current_model.size(); i++) {
        int currIdx = current_model[i].index;
        auto &cur_cell =
          this->polar_bin_grid->ang_cell[sector_index].lin_cell[currIdx];

        // go through all the points in this cell and assign to ground/not
        // ground
        for (unsigned int j = 0; j < cur_cell.bin_points.size(); j++) {
            float h =
              std::abs(current_model[i].height - cur_cell.bin_points[j].z);
            if (h < this->params.p_tg)  // z heights are close
            {
                ground_cloud->push_back(
                  cur_cell.bin_points[j]);  // add the point to ground
                cur_cell.ground_points.push_back(cur_cell.bin_points[j]);
            } else {
                // check drivability
                if (h > this->params.robot_height) {
                    // drivable
                    cur_cell.bin_points[j].drivable = 1;
                } else {
                    cur_cell.bin_points[j].drivable = 0;
                    drv_cloud->push_back(cur_cell.bin_points[j]);  // add to obs
                    cur_cell.drv_points.push_back(cur_cell.bin_points[j]);
                }
                this->obs_cloud->push_back(
                  cur_cell.bin_points[j]);  // add to obs
                cur_cell.obs_points.push_back(cur_cell.bin_points[j]);
                obs_sum += Vec3(cur_cell.bin_points[j].x,
                                cur_cell.bin_points[j].y,
                                cur_cell.bin_points[j].z);
                num_obs++;
            }
        }
        // mean of obs points
        cur_cell.obs_mean = obs_sum / num_obs;
    }

    // FIXME: WHY IS F_S < SIGPTR SOMETIMES?
    int i;
    if (sufficient_model) {
        // add all the obs points from the non ground classified pts
        for (i = 0; i < (int) sig_points.size(); i++) {
            auto &cur_cell = polar_bin_grid->ang_cell[sector_index]
                               .lin_cell[sig_points[i].index];

            for (int j = 0; j < (int) cur_cell.bin_points.size(); j++) {
                float h = std::abs(cur_cell.bin_points[j].z - f_s(i));
                // check drivability
                if (h > this->params.robot_height) {
                    // drivable
                    cur_cell.bin_points[j].drivable = 1;
                } else {
                    cur_cell.bin_points[j].drivable = 0;
                    drv_cloud->push_back(cur_cell.bin_points[j]);  // add to obs
                    cur_cell.drv_points.push_back(cur_cell.bin_points[j]);
                }
                obs_cloud->push_back(cur_cell.bin_points[j]);  // add to obs
                cur_cell.obs_points.push_back(cur_cell.bin_points[j]);
                obs_sum += Vec3(cur_cell.bin_points[j].x,
                                cur_cell.bin_points[j].y,
                                cur_cell.bin_points[j].z);
                num_obs++;
            }
            // mean of obs points
            cur_cell.obs_mean = obs_sum / num_obs;
        }
    } else {
        LOG_INFO("WARNING:Insufficient Model for angular slice");
    }
}

}  // namespace wave
