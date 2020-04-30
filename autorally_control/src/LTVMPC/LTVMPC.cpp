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
#include <chrono>

// cvxgen vectors
Vars vars;         //cvxgen output values x and u
Params params;     //Need to populate this structure with controller variables x0, Q, R, A, B, d, umax
Workspace work;    //internal solver states
Settings settings; //struct for MPC solver settings

namespace autorally_control
{
LTVMPC::LTVMPC(std::string prefix = "~") : nh(prefix)
{
  /* Setup Pub/Subs */
  mapCASub = nh.subscribe("/MAP_CA/mapCA", 1, &LTVMPC::Solve, this);
  chassisCommandPub = nh.advertise<autorally_msgs::chassisCommand>("chassisCommand", 1);
  runstopPub = nh.advertise<autorally_msgs::runstop>("runstop", 10);

  /* Init first control command to zero */
  command.steering = 0.0;      // first loop needs this initialized
  command.throttle = 0.0;      // first loop needs this initialized
  controllerUpdateRate = 0.01; // Try to achieve 100 Hz update rate

  // setup instance of map utility with gazebo simulation parameters
  util = Utilities(0);

  /* Setup external CVXGEN solver parameters */
  set_defaults();       //setup MPC Solver defaults from solve.c
  setup_indexing();     //Init structs for solver states from solve.c
  settings.verbose = 0; // Set this to 1 if you want to see the internal solver information

  /* Initialize Q and R matrices to Zero and get pointers to the eigen matrices */
  setMPCCost();
  getPointerstoEigen();

  /* Setup dynamic reconfigure pipeline and tie to callback function */
  dynamic_reconfigure::Server<LTVMPC_paramsConfig>::CallbackType cb; // Start up dynamic reconfigure server
  cb = boost::bind(&LTVMPC::ConfigCallback, this, _1, _2);
  m_dynServer.setCallback(cb);
  ros::Duration d = ros::Duration(1.0);
  d.sleep();
}

void LTVMPC::setMPCCost()
{
  /* Init Eigen matrices for Q cost and R control cost */
  m_lock.lock();

  m_Q = Eigen::MatrixXd::Zero(m_nX, m_nX);
  m_R = Eigen::MatrixXd::Zero(m_nU, m_nU);
  m_linPoints = Eigen::MatrixXd::Zero(m_N + 1, m_nX);

  m_lock.unlock();
}

void LTVMPC::getPointerstoEigen()
{
  x_out_ptr = x0.data();
  Q_ptr = m_Q.data();
  R_ptr = m_R.data();
}
void LTVMPC::ConfigCallback(const LTVMPC_paramsConfig &config, uint32_t level)
{
  /* Initialize Q and R cost matrix diagonals with custom parameters defined in the LTV_params.cfg file 
       (autorally_control/cfg/LTV_MPC_params.cfg)
    */

  speedCommand = config.speed;
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

void LTVMPC::LTVMPCcb()
{
  //auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < 8; ++i)
  {
    m_linPoints(0, i) = x0(i, 0); // Update the first linPoints to the current state/
  }
  if (x0(0, 0) <= 0.5 || std::isnan(command.steering))
  { // We use LTI.
    for (int k = 1; k <= m_N; ++k)
    {
      for (int i = 0; i < 8; ++i)
      {
        m_linPoints(k, i) = x0(i, 0); // Update the linPoints to the current state/
      }
    }
  }
  std::vector<std::future<void>> futures;

  for (int k = 0; k <= m_N; ++k)
  {
    futures.emplace_back(std::async(std::launch::async, &LTVMPC::asyncMatrixPopulation, this, k));
  }

  //  Populate parameter arrays for CVXGEN solver
  for (int i = 0; i < m_nX * m_nX; i++)
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

  // Retrieve confirmation that all threads finished
  std::for_each(futures.begin(), futures.end(), [](std::future<void> &ftr) {
    ftr.wait();
  });

  solve(); // CVXGEN solves the problem.

  command.steering = *vars.u[0];       //   get steering command from MPC outputs
  command.throttle = *(vars.u[0] + 1); //   get throttle command from MPC outputs

  // Update linPoints
  for (int k = 0; k < m_N; ++k)
  {
    for (int i = 0; i < 8; ++i)
    {
      m_linPoints(k, i) = *(vars.x[k + 1] + i);
    }
  }
  for (int i = 0; i < 8; ++i)
  {
    m_linPoints(m_N, i) = m_linPoints(m_N - 1, i);
  }
  /*
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dur = finish - start;
  std::cout << "CB loop execution time (ms): " << dur.count() << std::endl;
  */
}

void LTVMPC::populateParameterArrays(double (&A)[64], double (&B)[16], double (&d)[8],
                                     const Eigen::Matrix<double, 8, 8, Eigen::ColMajor> &eigenA, const Eigen::Matrix<double, 8, 2, Eigen::ColMajor> &eigenB, const Eigen::Matrix<double, 8, 1> &eigend)
{
  //Unpack Eigen matrices into flat C arrays for CVXGEN solver
  const double *A_ptr = eigenA.data();
  const double *B_ptr = eigenB.data();
  const double *d_ptr = eigend.data();

  //  Populate parameter arrays for CVXGEN solver
  for (int i = 0; i < 8 * 8; i++)
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

void LTVMPC::Solve(autorally_msgs::mapCA CA_states)
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

    LTVMPCcb();
    command.header.stamp = ros::Time::now();
    command.sender = "LTVMPC";
    command.frontBrake = 0.0;
    chassisCommandPub.publish(command);
    prevTime = time;
  }
}

void LTVMPC::asyncMatrixPopulation(int k)
{
  Eigen::Matrix<double, 8, 1> linP;
  for (int i = 0; i < 8; ++i)
  {
    linP(i, 0) = m_linPoints(k, i); // shouldnt need a mutex here since I'm just reading.
  }
  //std::cout << util.obtainCurvatureFromS(linP(7, 0)) <<  ", ";

  Eigen::Matrix<double, 8, 8, Eigen::ColMajor> tempA;
  Eigen::Matrix<double, 8, 2, Eigen::ColMajor> tempB;
  Eigen::Matrix<double, 8, 1> tempd;
  tempA.setZero();
  tempB.setZero();
  tempd.setZero();
  Vehicle.LinearizeDynamics(linP, control, util.obtainCurvatureFromS(linP(7, 0)), tempA, tempB, tempd);

  switch (k)
  {
    case 0: 
      populateParameterArrays(params.A_0, params.B_0, params.d_0, tempA, tempB, tempd);
      break;
    
    case 1: 
      populateParameterArrays(params.A_1, params.B_1, params.d_1, tempA, tempB, tempd);
      break;

    case 2: 
      populateParameterArrays(params.A_2, params.B_2, params.d_2, tempA, tempB, tempd);
      break;
    
    case 3: 
      populateParameterArrays(params.A_3, params.B_3, params.d_3, tempA, tempB, tempd);
      break;
  
    case 4: 
      populateParameterArrays(params.A_4, params.B_4, params.d_4, tempA, tempB, tempd);
      break;
    
    case 5: 
      populateParameterArrays(params.A_5, params.B_5, params.d_5, tempA, tempB, tempd);
      break;

    case 6: 
      populateParameterArrays(params.A_6, params.B_6, params.d_6, tempA, tempB, tempd);
      break;
    
    case 7: 
      populateParameterArrays(params.A_7, params.B_7, params.d_7, tempA, tempB, tempd);
      break;
  
    case 8: 
      populateParameterArrays(params.A_8, params.B_8, params.d_8, tempA, tempB, tempd);
      break;

    case 9: 
      populateParameterArrays(params.A_9, params.B_9, params.d_9, tempA, tempB, tempd);
      break;
    
    case 10: 
      populateParameterArrays(params.A_10, params.B_10, params.d_10, tempA, tempB, tempd);
      break;
  
    case 11: 
      populateParameterArrays(params.A_11, params.B_11, params.d_11, tempA, tempB, tempd);
      break;
    
    case 12: 
      populateParameterArrays(params.A_12, params.B_12, params.d_12, tempA, tempB, tempd);
      break;
  
  default:      
      populateParameterArrays(params.A_0, params.B_0, params.d_0, tempA, tempB, tempd);
      break;
  }
}

}; // namespace autorally_control

