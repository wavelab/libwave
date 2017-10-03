/************************************************************************
 *
 *
 *  Copyright 2015  Arun Das (University of Waterloo)
 *                      [adas@uwaterloo.ca]
 *                  James Servos (University of Waterloo)
 *                      [jdservos@uwaterloo.ca]
 *
 *
 *************************************************************************/
#include <Eigen/Eigenvalues>
#include <wave/matching/groundSegmentation.hpp>
#include <wave/matching/PointcloudXYZGD.hpp>

namespace wave {

bool compareSignalPoints(const signalPoint &a, const signalPoint &b) {
    return a.height < b.height;
}

groundSegmentation::groundSegmentation(GroundSegmentationParams config) {
    this->params = config;
    this->pBG = new polarBinGrid;
    initializePolarBinGrid();
}

void groundSegmentation::initializePolarBinGrid(void) {
    this->pBG->aCell.resize(this->params.num_bins_a);
    for (int i = 0; i < this->params.num_bins_a; i++) {
        this->pBG->aCell[i].sigPoints.clear();
        this->pBG->aCell[i].lCell.resize(this->params.num_bins_l);
        this->pBG->aCell[i].sigPoints.resize(this->params.num_bins_l);
        this->pBG->aCell[i].rangeHeightSignal.resize(this->params.num_bins_l);
        std::vector<signalPoint>().swap(pBG->aCell[i].sigPoints);
        for (int j = 0; j < this->params.num_bins_l; j++) {
            pBG->aCell[i].lCell[j].binPoints.clear();
            pBG->aCell[i].lCell[j].obsPoints.clear();
            pBG->aCell[i].lCell[j].drvPoints.clear();
            pBG->aCell[i].lCell[j].groundPoints.clear();
            pBG->aCell[i].lCell[j].prototypePoint = PointXYZGD();
            pBG->aCell[i].lCell[j].prototypePoint.x = NAN;
            pBG->aCell[i].lCell[j].prototypePoint.y = NAN;
            pBG->aCell[i].lCell[j].prototypePoint.z = NAN;
            pBG->aCell[i].rangeHeightSignal[j].x = NAN;
            pBG->aCell[i].rangeHeightSignal[j].y = NAN;
            pBG->aCell[i].lCell[j].cAssigned = -1;

            // force memory deletion of std::vectors
            std::vector<PointXYZGD>().swap(pBG->aCell[i].lCell[j].binPoints);
            std::vector<PointXYZGD>().swap(pBG->aCell[i].lCell[j].obsPoints);
            std::vector<PointXYZGD>().swap(pBG->aCell[i].lCell[j].drvPoints);
            std::vector<PointXYZGD>().swap(pBG->aCell[i].lCell[j].groundPoints);
        }
    }
}

void groundSegmentation::setupGroundSegmentation(
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
  pcl::PointCloud<PointXYZGD>::Ptr groundCloud,
  pcl::PointCloud<PointXYZGD>::Ptr obsCloud,
  pcl::PointCloud<PointXYZGD>::Ptr drvCloud) {
    groundCloud->clear();
    obsCloud->clear();

    refCloud = inputCloud;  // set the cloud datastructure;
    oCloud = obsCloud;      // set obs cloud
    gCloud = groundCloud;   // set ground cloud
    dCloud = drvCloud;

    oCloud->clear();
    gCloud->clear();
    dCloud->clear();

    genPolarBinGrid(refCloud);
}

void groundSegmentation::genPolarBinGrid(
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {
    initializePolarBinGrid();

    size_t nPts = inputCloud->size();
    double bsize_rad = (double) ((360.0) / this->params.num_bins_a);
    double bsize_lin = (double) this->params.rmax / this->params.num_bins_l;
    for (size_t i = 0; i < nPts; i++) {
        PointXYZGD curPt;
        float px = curPt.x = inputCloud->points[i].x;
        float py = curPt.y = inputCloud->points[i].y;
        float pz = curPt.z = inputCloud->points[i].z;

        if (sqrt(px * px + py * py + pz * pz) < this->params.rmax) {
            double ph = (atan2(py, px)) * (180 / M_PI);  // in degrees
            if (ph < 0)
                ph = 360.0 + ph;

            // bin into sector

            unsigned int bind_rad =
              static_cast<unsigned int>(ph / bsize_rad);  // got the radial bin

            if (bind_rad == this->params.num_bins_a) {
                bind_rad = 0;
            }

            // get the linear bin
            float xyDist = sqrt(px * px + py * py);

            unsigned int bind_lin = static_cast<unsigned int>(
              xyDist / bsize_lin);  // got the radial bin

            pBG->aCell[bind_rad].lCell[bind_lin].binPoints.push_back(curPt);
            // add the point to the bin
            // check the protoype point

            if (isnanf(pBG->aCell[bind_rad].lCell[bind_lin].prototypePoint.z) ||
                pz < pBG->aCell[bind_rad]
                       .lCell[bind_lin]
                       .prototypePoint.z)  // smallest by z
            {
                pBG->aCell[bind_rad].lCell[bind_lin].prototypePoint = curPt;
                pBG->aCell[bind_rad].rangeHeightSignal[bind_lin].x = xyDist;
                pBG->aCell[bind_rad].rangeHeightSignal[bind_lin].y = pz;
            }
        }
    }
}


MatX groundSegmentation::genGPModel(std::vector<signalPoint> &ps1,
                                    std::vector<signalPoint> &ps2,
                                    float sig_f,
                                    float p_l) {
    size_t nP1 = ps1.size();
    size_t nP2 = ps2.size();

    MatX CMAT;
    CMAT.resize(nP1, nP2);
    float coeff = (-1 / (2 * p_l * p_l));

    for (size_t i = 0; i < nP1; i++) {
        for (size_t j = 0; j < nP2; j++) {
            double diff = (ps1[i].range - ps2[j].range);
            CMAT(i, j) = sig_f * exp(coeff * (diff * diff));
        }
    }
    return CMAT;
}

void groundSegmentation::segmentGround() {
    for (int i = 0; i < this->params.num_bins_a; i++) {
        sectorINSAC(i);
    }
}

void groundSegmentation::sectorINSAC(int sectorIndex) {
    if (sectorIndex >= this->params.num_bins_a) {
        return;
    }
    int numFilled = 0;

    // pull out the valid points from the sector
    std::vector<signalPoint> &sigPtr = pBG->aCell[sectorIndex].sigPoints;
    sigPtr.clear();
    for (int i = 0; i < this->params.num_bins_l; i++) {
        if (!std::isnan(pBG->aCell[sectorIndex].rangeHeightSignal[i].x) &&
            pBG->aCell[sectorIndex].lCell[i].binPoints.size() > 5) {
            // bin has a valid point, and enough points to make a good
            // guess for a protopoint
            signalPoint newPoint;
            newPoint.range = pBG->aCell[sectorIndex].rangeHeightSignal[i].x;
            newPoint.height = pBG->aCell[sectorIndex].rangeHeightSignal[i].y;
            newPoint.idx = i;
            sigPtr.push_back(newPoint);
            numFilled++;
        }
    }
    // get the seed points.  Select the 3 lowest points.  Sort based on height
    // values
    sort(sigPtr.begin(), sigPtr.end(), compareSignalPoints);

    // now that the z points are sorted by height, take the
    // this->params.num_seed_points worth
    // as the seed
    size_t npt =
      sigPtr.size() < static_cast<size_t>(this->params.num_seed_points)
        ? sigPtr.size()
        : static_cast<size_t>(this->params.num_seed_points);
    std::vector<signalPoint> currentModel;
    int ptCtr = 0;
    int currIdx = 0;
    bool keepGoing = true;
    bool sufficientModel = true;

    while (true) {
        if (static_cast<size_t>(currIdx) >= sigPtr.size())  // overflow
        {
            break;
        }

        if (sigPtr[currIdx].range < this->params.max_seed_range &&
            fabs(sigPtr[currIdx].height) < this->params.max_seed_height) {
            // close enough to
            // robot and height
            // makese sense in
            // robot locality

            sigPtr[currIdx].isGround = true;
            currentModel.push_back(sigPtr[currIdx]);
            sigPtr.erase(sigPtr.begin() + currIdx);
            ptCtr++;

        } else {
            currIdx++;
        }

        if (static_cast<size_t>(ptCtr) >= npt)  // done
        {
            break;
        }
    }

    // check size
    if (currentModel.size() < 2)  // not enough for model, all obs pts
    {
        keepGoing = false;
        sufficientModel = false;
    }

    // got the seedpoints, start theINSAC process
    // cov matrices
    MatX C_XsX;
    MatX C_XX;
    MatX C_XsXs;
    MatX C_XXs;

    if (sigPtr.size() == 0)
        // no points to insac, put the seed points in as ground
        keepGoing = false;

    MatX temp;
    MatX f_s;
    MatX Vf_s;
    while (keepGoing) {
        // generate the covariance matrices

        C_XsX =
          genGPModel(sigPtr, currentModel, this->params.p_sf, this->params.p_l);
        C_XX = genGPModel(
          currentModel, currentModel, this->params.p_sf, this->params.p_l);
        C_XsXs =
          genGPModel(sigPtr, sigPtr, this->params.p_sf, this->params.p_l);
        C_XXs = C_XsX.transpose();

        // temporary calc
        MatX tCalc1 =
          C_XX + (this->params.p_sn * MatX::Identity(C_XX.rows(), C_XX.cols()));
        MatX tCalc2 = C_XsX * tCalc1.inverse();

        // test the points against the current model

        MatX modelZ(currentModel.size(), 1);
        for (unsigned int i = 0; i < currentModel.size(); i++) {
            modelZ(i, 0) = currentModel[i].height;
        }

        f_s = tCalc2 * modelZ;
        Vf_s = C_XsXs - tCalc2 * C_XXs;

        if (Vf_s.rows() == 0) {
            keepGoing = false;
            LOG_INFO("WARNING BREAKING LOOP: VF_s does not exist");
            continue;
        }

        bool searchCandidatePoints = true;
        unsigned int k = 0;
        // test for inliers using INSAC algorithm
        int startSize = currentModel.size();  // beginning size of the model set
        while (searchCandidatePoints) {
            double vf = Vf_s(k, k);
            double met =
              (sigPtr[k].height - f_s(k)) / (sqrt(this->params.p_sn + vf * vf));

            if (vf < this->params.p_tmodel &&
                abs(met) < this->params.p_tdata) {  // we have an inlier!
                // add to model set
                currentModel.push_back(sigPtr[k]);
                // remove from sample set
                sigPtr.erase(sigPtr.begin() + k);

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

            if (sigPtr.size() == k) {
                searchCandidatePoints = false;
            }
        }

        int endSize = currentModel.size();  // end size of the model set
        if (startSize == endSize || sigPtr.size() == 0) {
            keepGoing = false;
        }
    }  // end INSAC

    // fill in the ground and obs pointclouds

    double numObs = 0;
    Vec3 obsSum(0, 0, 0);

    for (int i = 0; i < (int) currentModel.size(); i++) {
        int currIdx = currentModel[i].idx;
        linCell *curCell = &pBG->aCell[sectorIndex].lCell[currIdx];

        // go through all the points in this cell and assign to ground/not
        // ground
        for (unsigned int j = 0; j < curCell->binPoints.size(); j++) {
            float h = abs(currentModel[i].height - curCell->binPoints[j].z);
            if (h < this->params.p_tg)  // z heights are close
            {
                gCloud->push_back(
                  curCell->binPoints[j]);  // add the point to ground
                curCell->groundPoints.push_back(curCell->binPoints[j]);
            } else {
                // check drivability
                if (h > this->params.robot_height) {
                    // drivable
                    curCell->binPoints[j].drivable = 1;
                } else {
                    curCell->binPoints[j].drivable = 0;
                    dCloud->push_back(curCell->binPoints[j]);  // add to obs
                    curCell->drvPoints.push_back(curCell->binPoints[j]);
                }
                oCloud->push_back(curCell->binPoints[j]);  // add to obs
                curCell->obsPoints.push_back(curCell->binPoints[j]);
                obsSum += Vec3(curCell->binPoints[j].x,
                               curCell->binPoints[j].y,
                               curCell->binPoints[j].z);
                numObs++;
            }
        }
        // mean of obs points
        curCell->obsMean = obsSum / numObs;
    }

    // FIXME: WHY IS F_S < SIGPTR SOMETIMES?
    int i;
    if (sufficientModel) {
        // add all the obs points from the non ground classified pts
        for (i = 0; i < (int) sigPtr.size(); i++) {
            linCell *curCell = &pBG->aCell[sectorIndex].lCell[sigPtr[i].idx];

            for (int j = 0; j < (int) curCell->binPoints.size(); j++) {
                float h = abs(curCell->binPoints[j].z - f_s(i));
                // check drivability
                if (h > this->params.robot_height) {
                    // drivable
                    curCell->binPoints[j].drivable = 1;
                } else {
                    curCell->binPoints[j].drivable = 0;
                    dCloud->push_back(curCell->binPoints[j]);  // add to obs
                    curCell->drvPoints.push_back(curCell->binPoints[j]);
                }
                oCloud->push_back(curCell->binPoints[j]);  // add to obs
                curCell->obsPoints.push_back(curCell->binPoints[j]);
                obsSum += Vec3(curCell->binPoints[j].x,
                               curCell->binPoints[j].y,
                               curCell->binPoints[j].z);
                numObs++;
            }
            // mean of obs points
            curCell->obsMean = obsSum / numObs;
        }
    } else {
        LOG_INFO("WARNING:Insufficnent Model for angular slice");
    }
}

}  // namespace wave
