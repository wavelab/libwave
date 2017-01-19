#include "slam/viz/vizcamera.hpp"


namespace slam {

// void VizCamera::setPosition(
//     float pos_x,  float pos_y,  float pos_z,
//     float view_x, float view_y, float view_z,
//     float up_x,   float up_y,   float up_z
// )
// {
//     mPos = tVector3(pos_x,  pos_y,  pos_z );  // set position
//     mView = tVector3(view_x, view_y, view_z);  // set view
//     mUp = tVector3(up_x,   up_y,   up_z  );  // set the up vector
// }
//
// void VizCamera::Move_Camera(float speed)
// {
//     tVector3 vVector = mView - mPos; // Get the view vector
//
//     // forward positive camera speed and backward negative camera speed.
//     mPos.x  = mPos.x  + vVector.x * speed;
//     mPos.z  = mPos.z  + vVector.z * speed;
//     mView.x = mView.x + vVector.x * speed;
//     mView.z = mView.z + vVector.z * speed;
// }

}  // end of slam namespace
