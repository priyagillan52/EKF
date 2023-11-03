
#include "ekf_models.hpp"
#include <tf/tf.h>
#include "utilities.h"

/**
   TODO
   Fill in the value of the process covariance matrix. The rows/columns of WMWt are
   in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ].
   \param[out] WMWt Covariance matrix of the system.
   \param state_in    The current state estimate
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
void sys_evaluate_WMWt(double WMWt[6][6], const State &state, double v, double w, double dt)
{
  double x = state_in.x[0];
  double y = state_in.x[1];
  double z = state_in.x[2];
  double roll = state_in.x[3];
  double pitch = state_in.x[4];
  double yaw = state_in.x[5];
  double alpha1 = 1.0; 
  double alpha2 = 1.0; 
  double alpha3 = 1.0;
  double alpha4 = 1.0;

  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++)
      WMWt[r][c] = 0.0;

  // TODO fill in the matrix WMWt
  WMWt[0][0] = (dt * dt) * pow(cos(pitch), 2.0) * pow(cos(yaw), 2.0) * (alpha1 * (v * v) + alpha2 * (w * w));
  WMWt[0][1] = (dt * dt) * cos(pitch) * cos(yaw) * (cos(roll) * sin(yaw) + cos(yaw) * sin(pitch) * sin(roll)) * (alpha1 * (v * v) + alpha2 * (w * w));
  WMWt[0][2] = (dt * dt) * cos(pitch) * cos(yaw) * (sin(roll) * sin(yaw) - cos(roll) * cos(yaw) * sin(pitch)) * (alpha1 * (v * v) + alpha2 * (w * w));
  WMWt[1][0] = (dt * dt) * cos(pitch) * cos(yaw) * (cos(roll) * sin(yaw) + cos(yaw) * sin(pitch) * sin(roll)) * (alpha1 * (v * v) + alpha2 * (w * w));
  WMWt[1][1] = (dt * dt) * pow(cos(roll) * sin(yaw) + cos(yaw) * sin(pitch) * sin(roll), 2.0) * (alpha1 * (v * v) + alpha2 * (w * w));
  WMWt[1][2] = (dt * dt) * (cos(roll) * sin(yaw) + cos(yaw) * sin(pitch) * sin(roll)) * (sin(roll) * sin(yaw) - cos(roll) * cos(yaw) * sin(pitch)) * (alpha1 * (v * v) + alpha2 * (w * w));
  WMWt[2][0] = (dt * dt) * cos(pitch) * cos(yaw) * (sin(roll) * sin(yaw) - cos(roll) * cos(yaw) * sin(pitch)) * (alpha1 * (v * v) + alpha2 * (w * w));
  WMWt[2][1] = (dt * dt) * (cos(roll) * sin(yaw) + cos(yaw) * sin(pitch) * sin(roll)) * (sin(roll) * sin(yaw) - cos(roll) * cos(yaw) * sin(pitch)) * (alpha1 * (v * v) + alpha2 * (w * w));
  WMWt[2][2] = (dt * dt) * pow(sin(roll) * sin(yaw) - cos(roll) * cos(yaw) * sin(pitch), 2.0) * (alpha1 * (v * v) + alpha2 * (w * w));
  WMWt[5][5] = (dt * dt) * (alpha3 * (v * v) + alpha4 * (w * w));
}

/**
   TODO
   Fill in the value of the measurement covariance matrix. The rows/columns of C
   are in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ]
   \param[out] R Covariance matrix of the sensors.
   \param state_in    The current state estimate
*/
void meas_evaluate_R(double R[6][6], const State &state)
{
  // play around with these values
  double alpha_x = 1.0;
  double alpha_y = 1.0;
  double alpha_z = 1.0;
  double alpha_roll = 1.0;
  double alpha_pitch = 1.0;
  double alpha_yaw = 1.0;

  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++)
      R[r][c] = 0.0;

  // TODO fill in the matrix R
  R[0][0] = alpha_x;
  R[1][1] = alpha_y;
  R[2][2] = alpha_z;
  R[3][3] = alpha_roll;
  R[4][4] = alpha_pitch;
  R[5][5] = alpha_yaw;
}

/**
   TODO
   Evaluate the system function.
   Compute the process model.
   This function returns the prediction of the next state based on the
   current state estimate and the commmand input (linear/angular velocities).
   \param state_in    The current state estimate
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
State sys_evaluate_g(const State &state_in, double v, double w, double dt)
{

  State state_out;

  // TODO Given state_in and v and w and dt (time increment) determine the prior
  // estimate state_out
  double x = state_in.x[0];
  double y = state_in.x[1];
  double z = state_in.x[2];
  double roll = state_in.x[3];
  double pitch = state_in.x[4];
  double yaw = state_in.x[5];

  state_out.x[0] = x + dt * v * cos(pitch) * cos(yaw);
  state_out.x[1] = y + dt * v * (cos(roll) * sin(yaw) + cos(yaw) * sin(pitch) * sin(roll));
  state_out.x[2] = z + dt * v * (sin(roll) * sin(yaw) - cos(roll) * cos(yaw) * sin(pitch));
  state_out.x[3] = atan2(cos(pitch) * sin(roll), cos(pitch) * cos(roll));
  state_out.x[4] = asin(sin(pitch));
  state_out.x[5] = atan2(sin(yaw + dt * w) * cos(pitch), cos(yaw + dt * w) * cos(pitch));

  return state_out;
}

/**
   TODO
   Evaluate the system Jacobian.
   This function evaluates the Jacobian of the system functions g (see
   sys_evaluate_g). The entry G[i][j] represents ( d g_i / d s_j )
   \param[out] G      The 6x6 Jacobian of the function g
   \param state       The state of the robot
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
void sys_evaluate_G(double G[6][6], const State &state, double v, double w, double dt)
{

  double x = state_in.x[0];
  double y = state_in.x[1];
  double z = state_in.x[2];
  double roll = state_in.x[3];
  double pitch = state_in.x[4];
  double yaw = state_in.x[5];

  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++)
      G[r][c] = 0.0;

  // TODO
  // Given state, v and w, compute the system Jacobian G

  G[0][0] = 1.0;
  G[0][4] = -dt * v * cos(yaw) * sin(pitch);
  G[0][5] = -dt * v * cos(pitch) * sin(yaw);
  G[1][1] = 1.0;
  G[1][3] = -dt * v * (sin(roll) * sin(yaw) - cos(roll) * cos(yaw) * sin(pitch));
  G[1][4] = dt * v * cos(pitch) * cos(yaw) * sin(roll);
  G[1][5] = dt * v * (cos(roll) * cos(yaw) - sin(pitch) * sin(roll) * sin(yaw));
  G[2][2] = 1.0;
  G[2][3] = dt * v * (cos(roll) * sin(yaw) + cos(yaw) * sin(pitch) * sin(roll));
  G[2][4] = -dt * v * cos(pitch) * cos(roll) * cos(yaw);
  G[2][5] = dt * v * (cos(yaw) * sin(roll) + cos(roll) * sin(pitch) * sin(yaw));
  G[3][3] = 1.0;
  G[4][4] = cos(pitch) / fabs(cos(pitch));
  G[5][5] = 1.0;
}

/**
   TODO
   Evaluate the GPS observation function.
   This function returns the expected satellite fix given the state of the robot
   \param state The state estimate
   \return      A satellite navigation fix (only the latitute, longitude
                and altitude members are used)
*/
sensor_msgs::NavSatFix meas_evaluate_gps(const State &state)
{
  double x = state_in.x[0];
  double y = state_in.x[1];
  double z = state_in.x[2];

  sensor_msgs::NavSatFix nsf;

  // TODO
  // Given prior estimate state, determine the expected GPS measurement nsf
  nsf.yaw = sqrt(x * x + y * y + z * z) - 6.378E+6;
  nsf.roll = asin(z / (sqrt(x * x + y * y + z * z) - 6.378E+6));
  nsf.pitch = t0 = atan2(y, x);
  return nsf;
}

/**
   TODO
   Evaluate the IMU observation function.
   This function computes the expected imu orientation given the state of the
   robot.
   \param state_in The current state estimate
   \return         A inertial navigation unit measurement (only the orientation
                   member is used).
*/
sensor_msgs::RPY meas_evaluate_imu(const State &state)
{
  sensor_msgs::RPY rpy;
  double roll = state.x[3];
  double pitch = state.x[4];
  double yaw = state.x[5];
  // TODO
  // Given the prior estimate state, determine the expected RPY measurement rpy
  rpy.roll = roll;
  rpy.pitch = pitch;
  rpy.yaw = yaw;
  return rpy;
}

/**
    TODO
    Observation Jacobian of the GPS
    This function returns the 3x3 observation Jacobian of the GPS. Essentially,
    this is the Jacobian of your meas_evaluate_gps function.
    \param[out] Hgps The 3x3 GPS Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Hgps(double Hgps[3][3], const State &state)
{

  // TODO
  // Fill the Jacobian matrix Hgps of the GPS observations
}

/**
    Observation Jacobian of the IMU
    This function returns the 3x3 observation Jacobian of the IMU. Essentially,
    this is the Jacobian of your meas_evaluate_imu function.
    \param[out] Himu The 3x3 IMU Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Himu(double Himu[3][3], const State &state)
{

  // TODO
  // Fill the Jacobian matrix Himu of the IMU observations
  Himu[0][0] = 1.0;
  Himu[0][1] = 0.0;
  Himu[0][2] = 0.0;
  Himu[1][0] = 0.0;
  Himu[1][1] = 1.0;
  Himu[1][2] = 0.0;
  Himu[2][0] = 0.0;
  Himu[2][1] = 0.0;
  Himu[2][2] = 1.0;
}
