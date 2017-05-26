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
#include <iostream>
#include "ground_segmentation/groundSegmentation.h"
#include <Eigen/Eigenvalues>
#include <cassert>

using namespace std;
using namespace Eigen;

float iSqrt(float x)
{
    float xhalf = 0.5f*x;
    int i = *(int*)&x; // get bits for floating value
    i = 0x5f375a86- (i>>1); // gives initial guess y0
    x = *(float*)&i; // convert bits back to float
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
    return x;
}

inline double fast_exp(double y) {
    double d;
    *((int*)(&d) + 0) = 0;
    *((int*)(&d) + 1) = (int)(1512775 * y + 1072632447);
    return d;
}

bool compareSignalPoints(const signalPoint &a, const signalPoint &b)
{
    return a.height < b.height;
}

groundSegmentation::groundSegmentation()
{

    //constructor, set default values

    set_rmax(100.0);
    set_num_maxbinpoints(200);
    set_num_seedpoints(10);

    set_gp_lengthparameter(10);
    set_gp_covariancescale(1.0);
    set_gp_modelnoise(0.3);
    set_gp_groundmodelconfidence(5.0);
    set_gp_grounddataconfidence(5.0);
    set_gp_groundthreshold(0.3);

    set_robotheight(1.2);

    set_seeding_maxrange(50);
    set_seeding_maxheight(15);

    pBG = new polarBinGrid;
    initializePolarBinGrid();

}


void groundSegmentation::initializePolarBinGrid(void)
{

//#pragma omp parallel
    for(int i=0 ; i<NUMBINSA; i++)
    {
        pBG->aCell[i].sigPoints.clear();
        std::vector<signalPoint>().swap(pBG->aCell[i].sigPoints);
        //#pragma omp parallel
        for(int j=0; j< NUMBINSL; j++)
        {
            pBG->aCell[i].lCell[j].binPoints.clear();
            pBG->aCell[i].lCell[j].obsPoints.clear();
            pBG->aCell[i].lCell[j].drvPoints.clear();
            pBG->aCell[i].lCell[j].groundPoints.clear();
            pBG->aCell[i].lCell[j].prototypePoint = PointXYZGD();
            pBG->aCell[i].lCell[j].prototypePoint.x = INVALID;
            pBG->aCell[i].lCell[j].prototypePoint.y = INVALID;
            pBG->aCell[i].lCell[j].prototypePoint.z = INVALID;
            pBG->aCell[i].rangeHeightSignal[j].x = INVALID;
            pBG->aCell[i].rangeHeightSignal[j].y = INVALID;
            pBG->aCell[i].lCell[j].cAssigned = -1;

            //force memory deletion of vectors
            std::vector<PointXYZGD>().swap(pBG->aCell[i].lCell[j].binPoints);
            std::vector<PointXYZGD>().swap(pBG->aCell[i].lCell[j].obsPoints);
            std::vector<PointXYZGD>().swap(pBG->aCell[i].lCell[j].drvPoints);
            std::vector<PointXYZGD>().swap(pBG->aCell[i].lCell[j].groundPoints);
        }
    }

}

void groundSegmentation::setupGroundSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, pcl::PointCloud<PointXYZGD>::Ptr groundCloud, pcl::PointCloud<PointXYZGD>::Ptr obsCloud,pcl::PointCloud<PointXYZGD>::Ptr drvCloud)
{

    groundCloud->clear();
    obsCloud->clear();

    refCloud = inputCloud; //set the cloud datastructure;
    oCloud = obsCloud; //set obs cloud
    gCloud = groundCloud; //set ground cloud
    dCloud = drvCloud;

    oCloud->clear();
    gCloud->clear();
    dCloud->clear();

    genPolarBinGrid(refCloud);

}

void groundSegmentation::genPolarBinGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{

    initializePolarBinGrid();

    unsigned int nPts = inputCloud->size();
    double bsize_rad = (double)( (360.0) / NUMBINSA); //in degrees
    double bsize_lin = (double)RMAX / NUMBINSL; //in meters
    //#pragma omp parallel
    for(int i=0; i<nPts; i++)
    {
        PointXYZGD curPt;;
        double px= curPt.x = inputCloud->points[i].x;
        double py= curPt.y = inputCloud->points[i].y;
        double pz= curPt.z = inputCloud->points[i].z;

        if(sqrt(px*px + py*py + pz*pz) < RMAX)
        {

            double ph = (atan2(py,px))*(180/M_PI); //in degrees
            if(ph<0)
                ph=360.0+ph;

            //bin into sector

            unsigned int bind_rad = floor((ph) /bsize_rad); //got the radial bin

            assert(bind_rad<NUMBINSA);

            // get the linear bin
            double xyDist = (sqrt( px*px + py*py));

            unsigned int bind_lin = floor((xyDist) /bsize_lin); //got the radial bin
            assert(bind_lin<NUMBINSL);


            pBG->aCell[bind_rad].lCell[bind_lin].binPoints.push_back(curPt); //add the point to the bin
            //check the protoype point

            if (pz< pBG->aCell[bind_rad].lCell[bind_lin].prototypePoint.z) // smallest by z
            {
                pBG->aCell[bind_rad].lCell[bind_lin].prototypePoint = curPt;
                pBG->aCell[bind_rad].rangeHeightSignal[bind_lin].x = xyDist;
                pBG->aCell[bind_rad].rangeHeightSignal[bind_lin].y = pz;

            }
        }

    }
    /*for(int i =0; i < NUMBINSL; i++) {
                 dCloud->push_back(pBG->aCell[50].lCell[i].prototypePoint);
        }*/
}


MatrixXd groundSegmentation::genGPModel(vector<signalPoint>& ps1, vector<signalPoint>& ps2,float sig_f,float p_l)
{
    int nP1  = ps1.size();
    int nP2  = ps2.size();

    MatrixXd CMAT;
    CMAT.resize(nP1,nP2);
    float coeff = (-1/ (2*p_l*p_l));

    for(int i=0; i<nP1; i++)
    {
        for(int j=0; j< nP2; j++ )
        {
            double diff = 	( ps1[i].range - ps2[j].range);
            CMAT(i,j) = sig_f*exp( coeff*(diff*diff) );
        }

    }
    return CMAT;

}

void groundSegmentation::segmentGround()
{
    for(int i=0; i< NUMBINSA; i++)
    {
        sectorINSAC(i);
    }

}

void groundSegmentation::sectorINSAC(int sectorIndex)
{
    if(sectorIndex >= NUMBINSA)
    {
        assert(true);
        return;
    }
    int numFilled = 0;

    // pull out the valid points from the sector
    vector<signalPoint> &sigPtr = pBG->aCell[sectorIndex].sigPoints;
    sigPtr.clear();
    for(int i=0; i<NUMBINSL ; i++)
    {

        if(pBG->aCell[sectorIndex].rangeHeightSignal[i].x != INVALID && pBG->aCell[sectorIndex].lCell[i].binPoints.size() > 5) //bin has a valid point, and enough points to make a good guess for a protopoint
        {
            signalPoint newPoint;
            newPoint.range = pBG->aCell[sectorIndex].rangeHeightSignal[i].x;
            newPoint.height = pBG->aCell[sectorIndex].rangeHeightSignal[i].y;
            newPoint.idx = i;
            sigPtr.push_back(newPoint);
            numFilled++;

        }

    }


    //get the seed points.  Select the 3 lowest points.  Sort based on height values
    sort(sigPtr.begin(), sigPtr.end(),compareSignalPoints);

    //for(int i=0; i< sigPtr.size(); i++)
    //cout<< sigPtr[i].idx<< endl;

    //now that the z points are sorted by height, take the NUMSEEDPOINTS worth as the seed
    int npt = sigPtr.size() < NUMSEEDPOINTS ? sigPtr.size() : NUMSEEDPOINTS;
    vector<signalPoint> currentModel;
    int ptCtr = 0;
    int currIdx = 0;
    bool keepGoing = true;
    bool sufficientModel = true;

    while(true)
    {

        if(currIdx>=sigPtr.size()) //overflow
        {
            break;
        }

        if(sigPtr[currIdx].range < MAXSEEDRANGE && fabs(sigPtr[currIdx].height) < MAXSEEDHEIGHT) //close enough to robot and height makese sense in robot locality
        {
            sigPtr[currIdx].isGround = true;
            currentModel.push_back(sigPtr[currIdx]);
            sigPtr.erase (sigPtr.begin()+currIdx);
            ptCtr++;

        }
        else
            currIdx++;

        if(ptCtr>=npt) //done
        {
            break;
        }


        //cout<<sigPtr[i].height<<endl;;

    }

    //check size
    if(currentModel.size()<2) //not enough for model, all obs pts
    {
        keepGoing = false;
        sufficientModel = false;
        //cout<<"model too small!" <<endl;
    }

    //got the seedpoints, start theINSAC process


    //cov matrices
    MatrixXd C_XsX;
    MatrixXd C_XX;
    MatrixXd C_XsXs;
    MatrixXd C_XXs;

    if(sigPtr.size() ==0 ) //no points to insac, put the seed points in as ground
        keepGoing=false;

    MatrixXd temp;
    MatrixXd f_s;
    MatrixXd Vf_s;
    while(keepGoing)
    {
        //generate the covariance matrices

        C_XsX = genGPModel(sigPtr,currentModel,P_SF,P_L);
        C_XX =  genGPModel(currentModel,currentModel, P_SF,P_L);
        C_XsXs = genGPModel(sigPtr,sigPtr,P_SF,P_L);
        C_XXs = C_XsX.transpose();


        //temporary calc
        MatrixXd tCalc1 = C_XX + (P_SN * MatrixXd::Identity(C_XX.rows(),C_XX.cols() ));
        MatrixXd tCalc2 = C_XsX * tCalc1.inverse() ;

        //test the points against the current model

        MatrixXd modelZ(currentModel.size(),1);
        for(unsigned int i=0 ; i<currentModel.size(); i++)
        {
            modelZ(i,0) = currentModel[i].height;
        }

        f_s = tCalc2*modelZ;
        Vf_s = C_XsXs - tCalc2*C_XXs;

        if(Vf_s.rows() == 0){
            keepGoing = false;
            ROS_WARN_STREAM("WARNING BREAKING LOOP: VF_s does not exist"); //arun: are we still getting this?
            /*cout<< "debug output:"<<"current model size: "<<currentModel.size()<<"current sig size: " << sigPtr.size()<<endl;
            cout<< "XsX" << C_XsX <<endl;
            cout<< "XX" << C_XX <<endl;
            cout<< "XXs" << C_XXs <<endl;
            cout<<" end debug" <<endl;*/
            continue;
        }

        bool searchCandidatePoints = true;
        unsigned int k = 0;
        //test for inliers using INSAC algorithm
        int startSize =   currentModel.size(); //beginning size of the model set
        while(searchCandidatePoints)
        {

            double vf = Vf_s(k,k);
            double met = (sigPtr[k].height - f_s(k)) / ( sqrt(P_SN+vf*vf));
            //cout<<"model cov:"<<vf<<"model h: "<<met<<"actual h:"<<sigPtr[k].height<<endl;
            if( vf<P_TMODEL && abs(met)<P_TDATA) {//we have an inlier!

                //add to model set
                currentModel.push_back(sigPtr[k]);
                //remove from sample set
                sigPtr.erase(sigPtr.begin() + k);

                //delete row from f_s
                temp = f_s;
                f_s.resize(f_s.rows()-1,f_s.cols());
                f_s.topRows(k) = temp.topRows(k);
                f_s.bottomRows(temp.rows()-k-1) = temp.bottomRows(temp.rows()-k-1);

                //delete row from Vf_s
                temp = Vf_s;
                Vf_s.resize(Vf_s.rows()-1,Vf_s.cols());
                Vf_s.topRows(k) = temp.topRows(k);
                Vf_s.bottomRows(temp.rows()-k-1) = temp.bottomRows(temp.rows()-k-1);

                //delete col from Vf_s
                temp = Vf_s;
                Vf_s.resize(Vf_s.rows(),Vf_s.cols()-1);
                Vf_s.leftCols(k) = temp.leftCols(k);
                Vf_s.rightCols(temp.cols()-k-1) = temp.rightCols(temp.cols()-k-1);

            } else {
                k++;
            }

            if( sigPtr.size() == k)
                searchCandidatePoints=false;

        }

        int endSize = currentModel.size(); //end size of the model set
        //cout<<"endsize is :"<<endSize<<endl;

        if( startSize == endSize || sigPtr.size() == 0) //we added no new points in the current iteration, terminate INSAC
            keepGoing = false;
    } //end INSAC

    //fill in the ground and obs pointclouds

    double numObs = 0;
    Vector3d obsSum(0,0,0);

    //#pragma omp parallel
    for(int i=0; i<(int)currentModel.size(); i++)
    {
        int currIdx = currentModel[i].idx;
        linCell *curCell = &pBG->aCell[sectorIndex].lCell[currIdx];
        //cout<<"nPts is: "<< curCell->binPoints.size() <<"for idx: "<< currIdx<<endl;

        //go through all the points in this cell and assign to ground/not ground
        for(unsigned int j=0; j<curCell->binPoints.size(); j++)
        {
            float h = abs(currentModel[i].height - curCell->binPoints[j].z);
            if( h < P_TG) // z heights are close
            {
                gCloud->push_back(curCell->binPoints[j]); //add the point to ground
                curCell->groundPoints.push_back(curCell->binPoints[j]);
            }
            else
            {
                //check drivability
                if( h > ROBOT_HEIGHT) {
                    //drivable
                    curCell->binPoints[j].drivable = 1;
                } else {
                    curCell->binPoints[j].drivable = 0;
                    dCloud->push_back(curCell->binPoints[j]); //add to obs
                    curCell->drvPoints.push_back(curCell->binPoints[j]);
                }
                oCloud->push_back(curCell->binPoints[j]); //add to obs
                curCell->obsPoints.push_back(curCell->binPoints[j]);
                obsSum += Vector3d(curCell->binPoints[j].x,curCell->binPoints[j].y,curCell->binPoints[j].z) ;
                numObs++;
            }

        }

        //mean of obs points
        curCell->obsMean = obsSum/numObs;


    }

    //#pragma omp parallel
    //FIXME: WHY IS F_S < SIGPTR SOMETIMES?
    int i;
    //cout << "[Local Mapping] WARNING: f_s.size < sigPtr.size, wtf?" << " fsize: " << f_s.size() << " sPtr: " << sigPtr.size() << " did INSAC "<< sufficientModel << endl;

    if(sufficientModel)
    {
        for(i=0; i<(int)sigPtr.size(); i++) //add all the obs points from the non ground classified pts
        {
            linCell *curCell = &pBG->aCell[sectorIndex].lCell[sigPtr[i].idx];

            //#pragma omp parallel
            for(int j=0; j< (int)curCell->binPoints.size(); j++)
            {
                float h = abs(curCell->binPoints[j].z - f_s(i));
                //check drivability
                if( h > ROBOT_HEIGHT) {
                    //drivable
                    curCell->binPoints[j].drivable = 1;
                } else {
                    curCell->binPoints[j].drivable = 0;
                    dCloud->push_back(curCell->binPoints[j]); //add to obs
                    curCell->drvPoints.push_back(curCell->binPoints[j]);
                }
                oCloud->push_back(curCell->binPoints[j]); //add to obs
                curCell->obsPoints.push_back(curCell->binPoints[j]);
                obsSum += Vector3d(curCell->binPoints[j].x,curCell->binPoints[j].y,curCell->binPoints[j].z) ;
                numObs++;
            }


            //mean of obs points
            curCell->obsMean = obsSum/numObs;

        }
    }
    else{
        ROS_DEBUG_STREAM("WARNING:Insufficnent Model for angular slice");
    }

    /*if(i==(int)f_s.size()) {
        cout << "[Local Mapping] WARNING: f_s.size < sigPtr.size, wtf?" << " fsize: " << f_s.size() << " sPtr: " << sigPtr.size() << endl;
    }*/

}
