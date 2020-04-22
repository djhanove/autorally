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
LTIMPC::LTIMPC() : m_nh("~")
{
  m_mapCASub = m_nh.subscribe("/MAP_CA/mapCA", 1, &LTIMPC::Solve, this);
  m_chassisCommandPub = m_nh.advertise<autorally_msgs::chassisCommand>("chassisCommand", 1);
  m_runstopPub = m_nh.advertise<autorally_msgs::runstop>("runstop", 10);
  m_trajectoryPub = m_nh.advertise<visualization_msgs::MarkerArray>( "MPC_Trajectory", 1 );
  command.steering = 0.0; // first loop needs this initialized
  command.throttle = 0.0; // first loop needs this initialized

  set_defaults();       //setup MPC Solver defaults from solve.c
  setup_indexing();     //Init structs for solver states from solve.c
  settings.verbose = 0; // Set this to 1 if you want to see the internal solver information


  setMPCCost();

  cb = boost::bind(&LTIMPC::ConfigCallback, this, _1, _2);
  m_dynServer.setCallback(cb);
  ros::Duration d = ros::Duration(1.0);
  d.sleep();
}

LTIMPC::~LTIMPC() {}

void LTIMPC::setMPCCost()
{

  /* Init Eigen matrices for Q cost and R control cost */

  m_lock.lock();

  m_Q = Eigen::MatrixXd::Zero(8, 8);
  m_R = Eigen::MatrixXd::Zero(2, 2);
  
  m_lock.unlock();
}

void LTIMPC::ConfigCallback(const LTIMPC_paramsConfig &config, uint32_t level)
{
  /* Initialize Q and R cost matrix diagonals with custom parameters defined in the LTI_params.cfg file 
       (autorally_control/cfg/LTI_MPC_params.cfg)
      Launch dynamic reconfigure to adjust cost function for R and Q matrices, as well as set speed target
      and road width params
  */

  speedCommand = config.speed;
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
  params.half_road_width[0] = config.Track_Width / 2.0;
}


void LTIMPC::LTIMPCcb()
{

  Vehicle.LinearizeDynamics(x0, control, curvature); 
  //  LTI linearizes around current state, LTV takes previous history of states and then linearizes
  //  At this point, system dynamics matrices
  //  m_A, m_B, and m_d are ready.
  //  x_{k+1} = Ax_k + Bu_k + d

  //Unpack Eigen matrices into flat C arrays for CVXGEN solver
  double *x_out_ptr = x0.data(); 
  double *A_ptr = Vehicle.m_A.data();
  double *B_ptr = Vehicle.m_B.data();
  double *d_ptr = Vehicle.m_d.data();
  double *Q_ptr = m_Q.data();
  double *R_ptr = m_R.data();

  params.target[0] = speedCommand; //Vx
  params.target[1] = 0.0; //  Vy - Targeting Vy = 0 seems like it may adversely impact controller performance pending mu characteristics
  params.target[2] = 0.0; //  Yaw Angle
  params.target[3] = speedCommand / Vehicle.getFrontWheelRadius(); //  Front wheel speeds
  params.target[4] = speedCommand / Vehicle.getRearWheelRadius(); //  Rear wheel speeds
  params.target[5] = 0.0; //  heading deviation 
  params.target[6] = 0.0; //  lateral deviation 
  params.target[7] = 0.0; //

  //  Populate parameter arrays for CVXGEN solver
  for (int i = 0; i < 8 * 8; i++)
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
  params.umax[0] = 1.00;
  params.umax[1] = 1.00;

  solve(); 

  command.steering = *vars.u[0];       //   get steering command from MPC outputs
  command.throttle = *(vars.u[0] + 1); //   get throttle command from MPC outputs

}

void LTIMPC::Solve(const autorally_msgs::mapCA &CA_states)
{
  
  /* Populate Initial State Vector for t_0 */
  x0(0, 0) = CA_states.vx;
  x0(1, 0) = CA_states.vy;
  x0(2, 0) = CA_states.wz;
  x0(3, 0) = CA_states.wf / Vehicle.getFrontWheelRadius();
  x0(4, 0) = CA_states.wr / Vehicle.getRearWheelRadius();
  x0(5, 0) = CA_states.epsi;
  x0(6, 0) = CA_states.ey;
  x0(7, 0) = CA_states.ey;

  /* Populate input matrix with control command from previous time step */
  control(0, 0) = command.steering;
  control(1, 0) = command.throttle;   

  curvature = CA_states.curvature;
  time = ros::Time::now().toSec();
  dt = time - m_prevTime;
  if (dt > .01)
  {
    // Gpscb(position);
    LTIMPCcb();
    command.header.stamp = ros::Time::now();
    command.sender = "LTIMPC";
    command.frontBrake = 0.0;
    m_chassisCommandPub.publish(command);
    m_prevTime = time;
  }
}

/*
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
*/


}; // namespace autorally_control

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LTIMPC");
  autorally_control::LTIMPC path_following_LTI_MPC;
  ros::spin();
}
