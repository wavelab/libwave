#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras.hpp>
namespace aslam {
namespace cameras {

CameraGeometryBase::CameraGeometryBase() {
}
;
CameraGeometryBase::~CameraGeometryBase() {
}
;

bool CameraGeometryBase::hasMask() const
{
	SM_THROW(std::runtime_error, "not implemented!");
	return false;
}


}  // namespace cameras
}  // namespace aslam

