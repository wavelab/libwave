# wave/matching

This module contains classes to perform scan matching/registration. There are (or
    will be) a variety of different scan matching algorithms available with a
    similar interface.

## Matcher

    template <typename T>
    class Matcher {
     public:
        Matcher(float res) : resolution(res) {}
        Matcher() {
            resolution = -1;
        }

        const Eigen::Affine3d &getResult() {
            return this->result;
        };
        const Eigen::MatrixXd &getInfo() {
            return this->information;
        };
        float getRes() {
            return this->resolution;
        };

        virtual void setRef(const T &ref) = 0;
        virtual void setTarget(const T &target) = 0;
        void setup(const T &ref, const T &target) {
            this->setRef(ref);
            this->setTarget(target);
        };

        virtual bool match() {
            return 0;
        }

     protected:
        float resolution;
        Eigen::Affine3d result;
        Eigen::MatrixXd information;
    };

    }  // end of wave namespace

This template class is inherited by each of the different matching algorithms.

### Members

    float resolution
The edge length (in the same distance-units as those used in the pointcloud) of
a downsampling voxel filter. If no downsampling is happening, this will be -1

---

    Eigen::Affine3d result
This is the transformation calculated by the scan registration algorithm. It is
the transformation needed to transform the target pointcloud to the reference
pointcloud in the reference frame of the reference pointcloud.

---

    Eigen::MatrixXd information

This is the information matrix calculated by the scan-matching algorithm. The
diagonal elements correpond to translational perturbations in the x, y, and z
directions and angular perturbations on the 3-2-1 euler angles, in that order
and in the frame of the reference pointcloud. This may change as the kinematics
 module of libwave progresses.

---

### Constructor

    Matcher()
This constructor will set up the instance of the matcher to attempt to match
using the full resolution of each point cloud (if possible).

---

    Matcher(float res)

This constructor takes an argument in order to adjust how much downsampling is
done before matching is attempted. Pointclouds are downsampled using a voxel filter,
the argument is the edge length of each voxel.

---

### Methods

    virtual void setRef(const T &ref) = 0;
    virtual void setTarget(const T &target) = 0;
    void setup(const T &ref, const T &target) {
        this->setRef(ref);
        this->setTarget(target);
    };

setRef and setTarget are implemented for each specific matching algorithm and are
how the pointclouds are passed to the matching object. Note that there isn't a
parameter for an initial transform; the initial transform is always assumed to
be identity inside the matcher class. If an initial transform estimate is available,
the target pointcloud should be transformed by it before being passed to the class
as some algorithms require a good initial estimate to perform well.

---

    virtual bool match();

Actually performs the match. Any heavy processing is done here.
Returns:
*  true if match was successful
*  false if match was not successful
