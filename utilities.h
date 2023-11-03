#include <tf/tf.h>

/** Convert a rotation matrix to Roll Pitch Yaw angles.
    according to RPY = Rx(R) * Ry(P) * Rz(Y)
 */
void rotm2rpy( const tf::Matrix3x3& RPY, double& R, double& P, double& Y );

/** Convert a Roll Pitch Yaw angles to a rotation matrix
    according to RPY = Rx(R) * Ry(P) * Rz(Y)
 */
tf::Matrix3x3 rpy2rotm( double R, double P, double Y );

