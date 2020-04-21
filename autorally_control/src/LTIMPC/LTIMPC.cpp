extern "C"
{
#include "solver.h"
}
#include "LTIMPC.h"
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>



// cvxgen vectors
Vars vars;         //cvxgen output values x and u
Params params;     //Need to populate this structure with controller variables x0, Q, R, A, B, d, umax
Workspace work;    //internal solver states
Settings settings; //struct for MPC solver settings

namespace autorally_control
{
LTIMPC::LTIMPC() : m_nh("~"), m_speed(0.0), m_N(12)
{
  // m_speedSub = m_nh.subscribe("/wheelSpeeds", 1, &LTIMPC::WheelSpeedcb, this);
  // if (m_usePoseEstimate)
  // 	m_odomSub = m_nh.subscribe("/pose_estimate", 1, &LTIMPC::Solve, this);
  // else
  // 	m_odomSub = m_nh.subscribe("/ground_truth/state", 1, &LTIMPC::Solve, this);
  m_mapCASub = m_nh.subscribe("/MAP_CA/mapCA", 1, &LTIMPC::Solve, this);
  m_chassisCommandPub = m_nh.advertise<autorally_msgs::chassisCommand>("chassisCommand", 1);
  m_runstopPub = m_nh.advertise<autorally_msgs::runstop>("runstop", 10);
  m_trajectoryPub = m_nh.advertise<visualization_msgs::MarkerArray>( "MPC_Trajectory", 1 );
  command.steering = 0.0; // first loop needs this initialized
  command.throttle = 0.0; // first loop needs this initialized

  set_defaults();       //setup MPC Solver defaults from solve.c
  setup_indexing();     //Init structs for solver states from solve.c
  settings.verbose = 0; // Set this to 1 if you want to see the internal solver information

  dynamic_reconfigure::Server<LTIMPC_paramsConfig>::CallbackType cb; // Start up dynamic reconfigure server

  setMPCCost();

  cb = boost::bind(&LTIMPC::ConfigCallback, this, _1, _2);
  m_dynServer.setCallback(cb);
  ros::Duration d = ros::Duration(1.0);
  d.sleep();
}

LTIMPC::~LTIMPC() {}

// void LTIMPC::Gpscb(nav_msgs::Odometry position)
// {
//   /*    Get state information from Gazebo odometry. Transform twist message from global to robot frame. 
//         Feed cartesian Gazebo coordinates into map function to return distance traveled along path (s),
//         lateral deviation from center line (m_n), and heading deviation relative to path abscissa (m_epsi)
//     */

//   double speed;
//   double x, y, theta;
//   m_lock.lock();
//   m_prevPos = m_position;
//   m_position = position;
//   speed = m_speed;
//   x = position.pose.pose.position.x;
//   y = position.pose.pose.position.y;

//   // Get our heading from the message
//   double roll, pitch, yaw;
//   tf::Quaternion quat(position.pose.pose.orientation.x,
//                       position.pose.pose.orientation.y,
//                       position.pose.pose.orientation.z,
//                       position.pose.pose.orientation.w);
//   tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

//   double deltaX = x - m_prevPos.pose.pose.position.x;
//   double deltaY = y - m_prevPos.pose.pose.position.y;
//   double thetaGPS = atan2(deltaY, deltaX);
//   m_lock.unlock();

//   if (m_useThetaGPS)
//     theta = thetaGPS;
//   else
//     theta = yaw;

//   if (theta < -M_PI)
//   {
//     theta = 2 * M_PI + theta;
//   }
//   else if (theta > M_PI)
//   {
//     theta = theta - 2 * M_PI;
//   }

//   map_instance.compute_ca_state(x, y, theta); //See map.h for logic taking place. map_instance is instantiated as a child class to LTIMPC
//   m_s = map_instance.c_state.s;
//   m_n = map_instance.c_state.n;
//   m_epsi = map_instance.c_state.d_psi;

//   m_vx = position.twist.twist.linear.x * cos(theta) + position.twist.twist.linear.y * sin(theta);  //transforms to robot frame
//   m_vy = position.twist.twist.linear.x * -sin(theta) + position.twist.twist.linear.y * cos(theta); //transforms to robot frame
//   m_wz = position.twist.twist.angular.z;
// }

void LTIMPC::setMPCCost()
{

  /* Init Eigen matrices for Q cost and R control cost */

  m_lock.lock();

  m_Q = Eigen::MatrixXd::Zero(Vehicle.m_nx, Vehicle.m_nx);
  m_R = Eigen::MatrixXd::Zero(Vehicle.m_nu, Vehicle.m_nu);
  
  m_lock.unlock();
}

// void LTIMPC::WheelSpeedcb(autorally_msgs::wheelSpeeds speeds)
// {
//   /*  Take average of left and ride wheel speeds for front/rear
//      */
//   m_lock.lock();
//   m_frontWheelsSpeed = (speeds.lfSpeed + speeds.rfSpeed) / 2.0;
//   m_rearWheelsSpeed = (speeds.lbSpeed + speeds.rbSpeed) / 2.0;
//   m_lock.unlock();
// }
void LTIMPC::ConfigCallback(const LTIMPC_paramsConfig &config, uint32_t level)
{
  /* Initialize Q and R cost matrix diagonals with custom parameters defined in the LTI_params.cfg file 
       (autorally_control/cfg/LTI_MPC_params.cfg)
    */

  m_speedCommand = config.speed;
  std::cout << "Target Speed Updated" << std::endl;

  m_Q(0, 0) = config.Q_vx;   //vx
  m_Q(1, 1) = config.Q_vy;   // vy
  m_Q(2, 2) = config.Q_wz;   //  wz
  m_Q(3, 3) = config.Q_wF;   // wF
  m_Q(4, 4) = config.Q_wR;   // wR
  m_Q(5, 5) = config.Q_epsi; //epsi
  m_Q(6, 6) = config.Q_ey;   // ey
  m_Q(7, 7) = config.Q_s;    // s

  m_R(0, 0) = config.R_delta; // steering
  m_R(1, 1) = config.R_T;     // Throttle
  // m_usePoseEstimate = config.use_pose_estimate; // To use pose estimate from state estimator
  params.half_road_width[0] = config.Track_Width / 2.0;
}

Eigen::Matrix<double, 8, 1> LTIMPC::computeState()
{
  /* Populate state matrix with current state */
  Eigen::Matrix<double, 8, 1> state;
  state(0, 0) = m_vx;
  state(1, 0) = m_vy;
  state(2, 0) = m_wz;
  state(3, 0) = m_frontWheelsSpeed / Vehicle.m_Vehicle_rF;
  state(4, 0) = m_rearWheelsSpeed / Vehicle.m_Vehicle_rR;
  state(5, 0) = m_epsi;
  state(6, 0) = m_n;
  state(7, 0) = m_s;
  return state;
}

Eigen::Matrix<double, 2, 1> LTIMPC::computeInput()
{
  /* Populate input matrix with current control */
  Eigen::Matrix<double, 2, 1> input;
  input(0, 0) = double(command.steering);
  input(1, 0) = double(command.throttle);
  //input(0,0) = -0.5;
  //input(1,0) = 0.25;
  
  return input;
}

void LTIMPC::LTIMPCcb()
{
  Eigen::Matrix<double, 8, 1> cur_state = computeState();
  Eigen::Matrix<double, 2, 1> input = computeInput();
  // cur = map_instance.obtainCurvature(cur_state(7,0)); // Obtain current road curvature
  Vehicle.LinearizeDynamics(cur_state, input, cur); 
  //  LTI linearizes around current state, LTV takes previous history of states and then linearizes
  //  At this point, system dynamics matrices
  //  m_A, m_B, and m_d are ready.
  //  x_{k+1} = Ax_k + Bu_k + d

  //Unpack Eigen matrices into flat C arrays for CVXGEN solver
  double *x_out_ptr = cur_state.data(); 
  double *A_ptr = Vehicle.m_A.data();
  double *B_ptr = Vehicle.m_B.data();
  double *d_ptr = Vehicle.m_d.data();
  double *Q_ptr = m_Q.data();
  double *R_ptr = m_R.data();

  params.target[0] = m_speedCommand; //Vx
  params.target[1] = 0.0; //  Vy - Targeting Vy = 0 seems like it may adversely impact controller performance pending mu characteristics
  params.target[2] = 0.0; //  Yaw Angle
  params.target[3] = m_speedCommand / Vehicle.m_Vehicle_rF; //  Front wheel speeds
  params.target[4] = m_speedCommand / Vehicle.m_Vehicle_rR; //  Rear wheel speeds
  params.target[5] = 0.0; //  heading deviation 
  params.target[6] = 0.0; //  lateral deviation 
  params.target[7] = 0.0; //

  //  Populate parameter arrays for CVXGEN solver
  for (int i = 0; i < Vehicle.m_nx * Vehicle.m_nx; i++)
  {
    if (i < 4)
    {
      params.x_0[i] = *(x_out_ptr + i);
      params.A[i] = *(A_ptr + i);
      params.B[i] = *(B_ptr + i);
      params.d[i] = *(d_ptr + i);
      params.Q[i] = *(Q_ptr + i);
      params.R[i] = *(R_ptr + i);
      params.QT[i] = 0.0;
    }
    else if (i < 8)
    {
      params.x_0[i] = *(x_out_ptr + i);
      params.A[i] = *(A_ptr + i);
      params.B[i] = *(B_ptr + i);
      params.d[i] = *(d_ptr + i);
      params.Q[i] = *(Q_ptr + i);
      params.QT[i] = 0.0;
    }
    else if (i < 16)
    {
      params.A[i] = *(A_ptr + i);
      params.B[i] = *(B_ptr + i);
      params.Q[i] = *(Q_ptr + i);
      params.QT[i] = 0.0;
    }
    else
    {
      params.A[i] = *(A_ptr + i);
      params.Q[i] = *(Q_ptr + i);
      params.QT[i] = 0.0;
    }
  }
  //  Set actuator limits for steer and brake commands (normalized)
  params.umax[0] = 0.99;
  params.umax[1] = 0.99;

  solve(); 
  // std::cout << "steering: " << *vars.u[0] << "throttle: " << *(vars.u[0] + 1) << std::endl;

  //std::cout << "throttle command sequence: ";
  // for(int i = 0; i <= 12; ++i){
  //   std::cout << *(vars.u[i]+1) << ", ";
  // }
  // std::cout << std::endl;
  
  // std::cout << "predited speed: ";
  // for(int i = 1; i <= 13; ++i){
  //   std::cout << *(vars.x[i]) << ", ";
  // }
  // std::cout<< std::endl;
  command.steering = *vars.u[0];       //   get steering command from MPC outputs
  command.throttle = *(vars.u[0] + 1); //   get throttle command from MPC outputs
  //command.steering = -0.5;
  //command.throttle = 0.25;
}

void LTIMPC::Solve(autorally_private_msgs::mapCA CA_states)
{
  /* Acts as a run function. Pulls everything together and publishes the control command to the platform */
  m_s = CA_states.s;
  m_n = CA_states.ey;
  m_epsi = CA_states.epsi;

  m_vx = CA_states.vx;
  m_vy = CA_states.vy;
  m_wz = CA_states.wz;

  m_frontWheelsSpeed = CA_states.wf;
  m_rearWheelsSpeed = CA_states.wr;

  cur = CA_states.curvature;
  std::cout << "ey: " << m_n << " epsi: " << m_epsi << " wz: " << m_wz << " vy: " << m_vy << std::endl;
  time = ros::Time::now().toSec();
  dt = time - m_prevTime;
  int i = 0;
  if (dt > .01)
  {
    // Gpscb(position);
    LTIMPCcb();
    command.header.stamp = ros::Time::now();
    command.sender = "LTIMPC";
    command.frontBrake = 0.0;
    m_chassisCommandPub.publish(command);
    m_prevTime = time;
    i++;
  }
  if (i % 10000 == 0)
  {
    ViewMPCTrajectory(CA_states.x, CA_states.y, CA_states.yaw);
  }
}

void LTIMPC::ViewMPCTrajectory(float state_est_x, float state_est_y, float state_est_yaw)
{

  marker.header.frame_id= "odom";
  marker.ns= "LTIMPC";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  // POINTS markers use x and y scale for width/height respectively
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  // Points are green
  marker.color.g = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(2.0);
  marker.header.stamp= ros::Time::now();
  marker.pose.position.x = (*vars.x[1] * cos(state_est_yaw) - (*(vars.x[1] + 1) * -sin(state_est_yaw))) * 0.1 + state_est_x;
  marker.pose.position.y = (*(vars.x[1] + 1) * sin(state_est_yaw) - (*vars.x[1] * cos(state_est_yaw)))  * 0.1 + state_est_y;
  float yaw = *(vars.x[1] + 2) * 0.1 + state_est_yaw;
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY( 0, 0, yaw );
  marker.pose.orientation.x = myQuaternion[0];
  marker.pose.orientation.y = myQuaternion[1];
  marker.pose.orientation.z = myQuaternion[2];
  marker.pose.orientation.w = myQuaternion[3];
  marker.id = 1;
  marker.action = 0;
  marker_array.markers.push_back(marker);
  for (int i = 2; i < 4; i++)
  {
    marker.header.stamp= ros::Time::now();
    marker.pose.position.x = (*vars.x[i] * cos(yaw) - (*(vars.x[i] + 1) * -sin(yaw)) * 0.1) + marker.pose.position.x;
    marker.pose.position.y = (*(vars.x[i] + 1) * sin(yaw) - (*vars.x[i] * cos(yaw))  * 0.1) + marker.pose.position.y;
    yaw = *(vars.x[i] + 2) * 0.1 + yaw;
    myQuaternion.setRPY( 0, 0, yaw );
    marker.pose.orientation.x = myQuaternion[0];
    marker.pose.orientation.y = myQuaternion[1];
    marker.pose.orientation.z = myQuaternion[2];
    marker.pose.orientation.w = myQuaternion[3];
    marker.id = i;
    marker_array.markers.push_back(marker);
  }
  m_trajectoryPub.publish(marker_array);
  marker_array.markers.clear();

}



}; // namespace autorally_control

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LTIMPC");
  autorally_control::LTIMPC path_following_LTI_MPC;
  ros::spin();
}
