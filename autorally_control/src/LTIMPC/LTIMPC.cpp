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
LTIMPC::LTIMPC(std::string prefix = "~") : nh(prefix)
{
  /* Setup Pub/Subs */
  mapCASub = nh.subscribe("/MAP_CA/mapCA", 1, &LTIMPC::Solve, this);
  chassisCommandPub = nh.advertise<autorally_msgs::chassisCommand>("chassisCommand", 1);
  runstopPub = nh.advertise<autorally_msgs::runstop>("runstop", 10);
  trajectoryPub = nh.advertise<visualization_msgs::MarkerArray>("MPC_Trajectory", 1);

  /* Init first control command to zero */
  command.steering = 0.0;      // first loop needs this initialized
  command.throttle = 0.0;      // first loop needs this initialized
  controllerUpdateRate = 0.01; // Try to achieve 100 Hz update rate

  /* Setup external CVXGEN solver parameters */
  set_defaults();       //setup MPC Solver defaults from solve.c
  setup_indexing();     //Init structs for solver states from solve.c
  settings.verbose = 0; // Set this to 1 if you want to see the internal solver information

  /* Initialize Q and R matrices to Zero */
  setMPCCost();

  /* Setup dynamic reconfigure pipeline and tie to callback function */
  cb = boost::bind(&LTIMPC::ConfigCallback, this, _1, _2);
  m_dynServer.setCallback(cb);
  ros::Duration d = ros::Duration(1.0);
  d.sleep(); // Wait 1 second for pub/sub and dynamic reconfigure connections to be established
}

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
  /*
    At this point, the system dynamics matrices
    A, B, and d are ready.
    x_{k+1} = Ax_k + Bu_k + d
  */

  // Set State Targets
  params.target[0] = speedCommand;                                 //Vx target
  params.target[1] = 0.0;                                          //Vy target
  params.target[2] = speedCommand * curvature;                     //  Yaw rate target
  params.target[3] = speedCommand / Vehicle.getFrontWheelRadius(); //  Front wheel speed target
  params.target[4] = speedCommand / Vehicle.getRearWheelRadius();  //  Rear wheel speeds target
  params.target[5] = 0.0;                                          //  heading deviation target
  params.target[6] = 0.0;                                          //  lateral deviation target
  params.target[7] = 0.0;                                          //  dist traveled target (cost is set to zero)

  //  Set actuator limits for steer and brake commands (normalized, abs value constraint)
  params.umax[0] = 0.99;
  params.umax[1] = 0.99;

  //  Populate parameter arrays for CVXGEN solver
  for (int i = 0; i < 8 * 8; i++)
  {
    if (i < 4)
    {
      params.x_0[i] = *(x0.data() + i);
      params.A[i] = *(Vehicle.m_A.data() + i);
      params.B[i] = *(Vehicle.m_B.data() + i);
      params.d[i] = *(Vehicle.m_d.data() + i);
      params.Q[i] = *(m_Q.data() + i);
      params.R[i] = *(m_R.data() + i);
      params.QT[i] = 0.0;
    }
    else if (i < 8)
    {
      params.x_0[i] = *(x0.data() + i);
      params.A[i] = *(Vehicle.m_A.data() + i);
      params.B[i] = *(Vehicle.m_B.data() + i);
      params.d[i] = *(Vehicle.m_d.data() + i);
      params.Q[i] = *(m_Q.data() + i);
      params.QT[i] = 0.0;
    }
    else if (i < 16)
    {
      params.A[i] = *(Vehicle.m_A.data() + i);
      params.B[i] = *(Vehicle.m_B.data() + i);
      params.Q[i] = *(m_Q.data() + i);
      params.QT[i] = 0.0;
    }
    else
    {
      params.A[i] = *(Vehicle.m_A.data() + i);
      params.Q[i] = *(m_Q.data() + i);
      params.QT[i] = 0.0;
    }
  }

  solve();
  command.steering = *vars.u[0];       //   get steering command from MPC outputs
  command.throttle = *(vars.u[0] + 1); //   get throttle command from MPC outputs
}

void LTIMPC::Solve(const autorally_msgs::mapCA &CA_states)
{
  time = ros::Time::now().toSec();
  dt = time - prevTime;
  if (dt > controllerUpdateRate)
  {
    /* Populate Initial State Vector for t_0 */
    x0(0, 0) = CA_states.vx;
    x0(1, 0) = CA_states.vy;
    x0(2, 0) = CA_states.wz;
    x0(3, 0) = CA_states.wf / Vehicle.getFrontWheelRadius();
    x0(4, 0) = CA_states.wr / Vehicle.getRearWheelRadius();
    x0(5, 0) = CA_states.epsi;
    x0(6, 0) = CA_states.ey;
    x0(7, 0) = CA_states.s;

    /* Populate input matrix with control command from previous time step */
    control(0, 0) = command.steering;
    control(1, 0) = command.throttle;

    curvature = CA_states.curvature;

    LTIMPCcb();
    command.header.stamp = ros::Time::now();
    command.sender = "LTIMPC";
    command.frontBrake = 0.0;
    chassisCommandPub.publish(command);
    prevTime = time;
  }
}

/*
Still need to figure out proper transforms back to world frame for visualization of MPC predictions
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

