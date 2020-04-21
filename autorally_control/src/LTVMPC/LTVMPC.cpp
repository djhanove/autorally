extern "C"
{
#include "solver.h"
}
#include "LTVMPC.h"
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
LTVMPC::LTVMPC() : m_nh("~"), m_speed(0.0)
{
  m_mapCASub = m_nh.subscribe("/MAP_CA/mapCA", 1, &LTVMPC::Solve, this);
  m_chassisCommandPub = m_nh.advertise<autorally_msgs::chassisCommand>("chassisCommand", 1);
  m_runstopPub = m_nh.advertise<autorally_msgs::runstop>("runstop", 10);
  command.steering = 0.0; // first loop needs this initialized
  command.throttle = 0.0; // first loop needs this initialized

  util = Utilities(1);
  for(int k = 0; k <= m_N; ++k){
    for(int i = 0; i < 8; ++i){
      m_linPoints[k][i] = -1000.0; // ToDo: More sophisticated initialization.
    }
  }

  set_defaults();       //setup MPC Solver defaults from solve.c
  setup_indexing();     //Init structs for solver states from solve.c
  settings.verbose = 0; // Set this to 1 if you want to see the internal solver information

  dynamic_reconfigure::Server<LTVMPC_paramsConfig>::CallbackType cb; // Start up dynamic reconfigure server

  setMPCCost();

  cb = boost::bind(&LTVMPC::ConfigCallback, this, _1, _2);
  m_dynServer.setCallback(cb);
  ros::Duration d = ros::Duration(1.0);
  d.sleep();
}

LTVMPC::~LTVMPC() {}

void LTVMPC::setMPCCost()
{
  /* Init Eigen matrices for Q cost and R control cost */
  m_lock.lock();

  m_Q = Eigen::MatrixXd::Zero(Vehicle.m_nx, Vehicle.m_nx);
  m_R = Eigen::MatrixXd::Zero(Vehicle.m_nu, Vehicle.m_nu);

  m_lock.unlock();
}

void LTVMPC::ConfigCallback(const LTVMPC_paramsConfig &config, uint32_t level)
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
  params.half_road_width[0] = config.Track_Width / 2.0;
}


Eigen::Matrix<double, 8, 1> LTVMPC::computeState()
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

Eigen::Matrix<double, 2, 1> LTVMPC::computeInput()
{
  /* Populate input matrix with current control */
  Eigen::Matrix<double, 2, 1> input;
  input(0, 0) = double(command.steering);
  input(1, 0) = double(command.throttle);
  return input;
}

void LTVMPC::LTVMPCcb()
{
  Eigen::Matrix<double, 8, 1> cur_state = computeState();
  Eigen::Matrix<double, 2, 1> input = computeInput();
  for(int i = 0; i < 8; ++i){
    m_linPoints[0][i] = cur_state(i,0);// Update the first linPoints to the current state/
  }
  if(m_vx <= 0.5 || m_linPoints[1][0] == -1000.0 || std::isnan( command.steering ) ){ // We use LTI.
    for(int k = 1; k <= m_N; ++k){
      for(int i = 0; i < 8; ++i){
        m_linPoints[k][i] = cur_state(i,0);// Update the linPoints to the current state/
      }
    }
  }
  std::cout << "Current s value: " << cur_state(7,0) << std::endl;
  std::cout << "Current Region: " << util.identifyRegionFromS(cur_state(7,0)) << std::endl;
  std::cout << "Road Curvature: " << util.obtainCurvatureFromS(cur_state(7,0)) << std::endl;
  for(int k = 0; k <= m_N; ++k){
    Eigen::Matrix<double, 8, 1> linP;
    // std::cout << "linP at k = " << k << std::endl;
    for(int i = 0; i < 8; ++i){
      linP(i,0) = m_linPoints[k][i];
    }
    std::cout << util.obtainCurvatureFromS(linP(7, 0)) <<  ", ";
    Vehicle.LinearizeDynamics(linP, input, util.obtainCurvatureFromS(linP(7, 0))); 
    //  LTV takes previous history of states and then linearizes
    //  ^ This statement was found to be wrong. LTV takes the predicted state from the previous time step as the linearization points. 
    // The following code apparently needs to be improved...
    if(k == 0){
      populateParameterArrays(params.A_0, params.B_0, params.d_0);
    } else if (k == 1){
      populateParameterArrays(params.A_1, params.B_1, params.d_1);
    } else if (k == 2){
      populateParameterArrays(params.A_2, params.B_2, params.d_2);
    } else if (k == 3){
      populateParameterArrays(params.A_3, params.B_3, params.d_3);
    } else if (k == 4){
      populateParameterArrays(params.A_4, params.B_4, params.d_4);
    } else if (k == 5){
      populateParameterArrays(params.A_5, params.B_5, params.d_5);
    } else if (k == 6){
      populateParameterArrays(params.A_6, params.B_6, params.d_6);
    } else if (k == 7){
      populateParameterArrays(params.A_7, params.B_7, params.d_7);
    } else if (k == 8){
      populateParameterArrays(params.A_8, params.B_8, params.d_8);
    } else if (k == 9){
      populateParameterArrays(params.A_9, params.B_9, params.d_9);
    } else if (k == 10){
      populateParameterArrays(params.A_10, params.B_10, params.d_10);
    } else if (k == 11){
      populateParameterArrays(params.A_11, params.B_11, params.d_11);
    } else { // k == 12
      populateParameterArrays(params.A_12, params.B_12, params.d_12);
    }
  }
  std::cout << std::endl;
  double *x_out_ptr = cur_state.data(); 
  double *Q_ptr = m_Q.data();
  double *R_ptr = m_R.data();
  
  //  Populate parameter arrays for CVXGEN solver
  for (int i = 0; i < Vehicle.m_nx * Vehicle.m_nx; i++)
  {
    if (i < 4)
    {
      params.x_0[i] = *(x_out_ptr + i);
      params.Q[i] = *(Q_ptr + i);
      params.R[i] = *(R_ptr + i);
      params.QT[i] = 0.0;
    }
    else if (i < 8)
    {
      params.x_0[i] = *(x_out_ptr + i);
      params.Q[i] = *(Q_ptr + i);
      params.QT[i] = 0.0;
    }
    else
    {
      params.Q[i] = *(Q_ptr + i);
      params.QT[i] = 0.0;
    }
  }
  //  Set actuator limits for steer and brake commands (normalized)
  params.umax[0] = 0.99;
  params.umax[1] = 0.99;
  
  params.target[0] = m_speedCommand; //Vx
  params.target[1] = 0.0; //  Vy - Targeting Vy = 0 seems like it may adversely impact controller performance pending mu characteristics
  params.target[2] = 0.0; //  Yaw Angle
  params.target[3] = m_speedCommand / Vehicle.m_Vehicle_rF; //  Front wheel speeds
  params.target[4] = m_speedCommand / Vehicle.m_Vehicle_rR; //  Rear wheel speeds
  params.target[5] = 0.0; //  heading deviation 
  params.target[6] = 0.0; //  lateral deviation 
  params.target[7] = 0.0; //

  solve(); // CVXGEN solves the problem.

  command.steering = *vars.u[0];       //   get steering command from MPC outputs
  command.throttle = *(vars.u[0] + 1); //   get throttle command from MPC outputs
  std::cout << "Steering Command: " << command.steering << ", " << "Throttle Command: " << command.throttle << std::endl;
  // Update linPoints
  for(int k = 0; k < m_N; ++k){
    for(int i = 0; i < 8; ++i){
      m_linPoints[k][i] = *(vars.x[k+1] + i);
    }
  }
  for(int i = 0; i < 8; ++i){
    m_linPoints[m_N][i] = m_linPoints[m_N-1][i];
  }
}

void LTVMPC::populateParameterArrays(double (&A)[64], double (&B)[16], double (&d)[8])
{
  //Unpack Eigen matrices into flat C arrays for CVXGEN solver
  double *A_ptr = Vehicle.m_A.data();
  double *B_ptr = Vehicle.m_B.data();
  double *d_ptr = Vehicle.m_d.data();
  
  //  Populate parameter arrays for CVXGEN solver
  for (int i = 0; i < Vehicle.m_nx * Vehicle.m_nx; i++)
  {
    if (i < 8)
    {
      A[i] = *(A_ptr + i);
      B[i] = *(B_ptr + i);
      d[i] = *(d_ptr + i);
    }
    else if (i < 16)
    {
      A[i] = *(A_ptr + i);
      B[i] = *(B_ptr + i);
    }
    else
    {
      A[i] = *(A_ptr + i);
    }    
  }
}

void LTVMPC::Solve(autorally_private_msgs::mapCA CA_states)
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
  time = ros::Time::now().toSec();
  dt = time - m_prevTime;
  if (dt > m_dt)
  {
    LTVMPCcb();
    command.header.stamp = ros::Time::now();
    command.sender = "LTVMPC";
    command.frontBrake = 0.0;
    m_chassisCommandPub.publish(command);
    m_prevTime = time;
  }
}


}; // namespace autorally_control

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LTVMPC");
  autorally_control::LTVMPC path_following_LTV_MPC;
  ros::spin();
}
