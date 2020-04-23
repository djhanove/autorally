#include <eigen3/Eigen/Dense>
#include <boost/thread.hpp>          // Mutex
#include <boost/lexical_cast.hpp>

class VehicleDynamics
{ public:
    static const int m_nx = 8;
    static const int m_nu = 2;
    VehicleDynamics();
    ~VehicleDynamics();
    // Vehicle Parameters
    float m_Vehicle_m;
    float m_Vehicle_Iz;
    float m_Vehicle_lF;
    float m_Vehicle_lR;
    float m_Vehicle_IwF;
    float m_Vehicle_IwR;
    float m_Vehicle_rF;
    float m_Vehicle_rR;
    float m_Vehicle_h;
    float m_g;
    float m_Vehicle_mu1;
    float m_Vehicle_mu2;

    double tire_B = 1.1559;
    double tire_C = 1.1924;
    double tire_D = 0.9956;
    double tire_E = -0.8505;
    double tire_Sh = -0.0540;
    double tire_Sv = 0.1444;
    
    // Linearized Vehicle Dynamics
    Eigen::Matrix<double, m_nx, m_nx, Eigen::ColMajor> m_A; 
    Eigen::Matrix<double, m_nx, m_nu, Eigen::ColMajor> m_B;
    Eigen::Matrix<double, m_nx, 1> m_d;
    
    boost::mutex m_lock;

    void setVehicleParameters();
    Eigen::Matrix<double, m_nx, 1> RWDBicycleModel(const Eigen::Matrix<double, m_nx, 1> &, const Eigen::Matrix<double, m_nu, 1> &, const double &); // Always pass by reference using Eigen matrices are fxn args
    double unnormalizeSteering(const double &);
    double unnormalizeThrottle(const double &);  
    void LinearizeDynamics(const Eigen::Matrix<double, m_nx, 1> &, const Eigen::Matrix<double, m_nu, 1>&,  const double &); // compute linearized dynamics

};

VehicleDynamics::VehicleDynamics()
{
    setVehicleParameters(); 
}

VehicleDynamics::~VehicleDynamics(){}

void VehicleDynamics::setVehicleParameters()
{
  /*  Initialize vehicle dynamics parameters for bicycle model and populate Eigen mats for A, B, and d */
  m_lock.lock();

  m_Vehicle_m = 21.88;//23.91;
  m_Vehicle_Iz = 1.124;//0.306744;
  m_Vehicle_lF = 0.34;
  m_Vehicle_lR = 0.23;
  m_Vehicle_IwF = 0.048;//0.003759;
  m_Vehicle_IwR = 0.044;//0.00408;
  m_Vehicle_rF = 0.095;
  m_Vehicle_rR = 0.095;
  m_Vehicle_mu1 = 0.7;  // Gazebo world parameter
  m_Vehicle_mu2 = 0.75; // Gazebo world parameter
  m_Vehicle_h = 0.2;    // Need to check this value
  m_g = 9.80665;
  m_A.setZero();
  m_B.setZero();
  m_d.setZero();

  m_lock.unlock();
}

Eigen::Matrix<double, 8, 1> VehicleDynamics::RWDBicycleModel(const Eigen::Matrix<double, m_nx, 1> &state, const Eigen::Matrix<double, m_nu, 1> &input, const double &curvature)
{
  /* Bicycle model with Pacejka tire model. Protects are included for when vehicle speed is near zero, as Pacejka models tend
    to output incorrect slip angle and force predictions when vel is near zero */

  double deltaT = 0.001;
  Eigen::Matrix<double, m_nx, 1> next_state;

  double delta = unnormalizeSteering(input(0, 0));
  double T = unnormalizeThrottle(input(1, 0));

  double vx = state(0, 0);
  double vy = state(1, 0);
  double wz = state(2, 0);
  double wF = state(3, 0);
  double wR = state(4, 0);
  double epsi = state(5, 0);
  double ey = state(6, 0);
  double s = state(7, 0);
  //std::cout << "Actual ey: " << ey << " Actual epsi: " << epsi << std::endl;
  int i = 1;
  while (i * deltaT <= 0.1) //Loop many times for more precise estimate using small dt
  {
    // Compute vertical forces on tire
    double fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * m_Vehicle_mu1) / (m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h * (m_Vehicle_mu1 * cos(delta) - m_Vehicle_mu2 * sin(delta) - m_Vehicle_mu1));
    double fRz = m_Vehicle_m * m_g - fFz;

    // Theoretical slip quantities
    double beta;
    if (vx < 0.2)
    {
      vx = 0.2; // To stabilize the dynamics
    }
    if (vx != 0.0)
    {
      beta = atan2(vy, vx);
    }
    else
    {
      beta = 0.0;
    }
    double V = sqrt(vx * vx + vy * vy); //Get vel vector
    double vFx = V * cos(beta - delta) + wz * m_Vehicle_lF * sin(delta);
    double vFy = V * sin(beta - delta) + wz * m_Vehicle_lF * cos(delta);
    double vRx = vx;
    double vRy = vy - wz * m_Vehicle_lR;
    double sFx, sRx;
    if (wF != 0.0)
    {
      sFx = (vFx - wF * m_Vehicle_rF) / (wF * m_Vehicle_rF);
    }
    else
    {
      sFx = 0.0;
    }
    if (wR != 0.0)
    {
      sRx = (vRx - wR * m_Vehicle_rR) / (wR * m_Vehicle_rR);
    }
    else
    {
      sRx = 0.0;
    }
    double sFy, sRy;
    if (vFx != 0.0)
    {
      sFy = (1 + sFx) * vFy / vFx;
    }
    else
    {
      sFy = 0.0;
    }
    if (vRx != 0.0)
    {
      sRy = (1 + sRx) * vRy / vRx;
    }
    else
    {
      sRy = 0.0;
    }
    double sF = sqrt(sFx * sFx + sFy * sFy);
    double sR = sqrt(sRx * sRx + sRy * sRy);

    double sEF = sF - tire_Sh;
    double sER = sR - tire_Sh;

    // double muF = tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(sEF) ) ) ) + tire_Sv;
    // double muR = tire_D*sin( tire_C*atan( tire_B*sER - tire_E*(tire_B*sER - atan(sER) ) ) ) + tire_Sv;

    double muF = tire_D * sin(tire_C * atan(tire_B * sF));
    double muR = tire_D * sin(tire_C * atan(tire_B * sR));
    
    // Compute Longitudinal and Lateral forces at front/rear tires
    double fFx = -sFx / sF * muF * fFz;
    double fFy = -sFy / sF * muF * fFz;
    double fRx = -sRx / sR * muR * fRz;
    double fRy = -sRy / sR * muR * fRz;
    if(i==100)
    {
     // std::cout << "sFx: " << sFx<< " sRx: " << sRx << " sFy: " << sFy << " sRy: " << sRy << " delta: " << delta << " iter: " << i << std::endl;
      //std::cout << "FFx: " << fFx<< " FRx: " << fRx << " FFy: " << fFy << " FRy: " << fRy << " iter: " << i << std::endl;
      //std::cout << T<< std::endl;
    }
    // compute next state using (V.6)
    next_state(0, 0) = vx + deltaT * ((fFx * cos(delta) - fFy * sin(delta) + fRx) / m_Vehicle_m + vy * wz);
    next_state(1, 0) = vy + deltaT * ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m - vx * wz);
    next_state(2, 0) = wz + deltaT * ((fFy * cos(delta) + fFx * sin(delta)) * m_Vehicle_lF - fRy * m_Vehicle_lR) / m_Vehicle_Iz;
    next_state(3, 0) = wF - deltaT * m_Vehicle_rF / m_Vehicle_IwF * fFx;
    next_state(4, 0) = wR + deltaT / m_Vehicle_IwR * (T - m_Vehicle_rR * fRx);
    next_state(5, 0) = epsi + deltaT * (wz - (vx * cos(epsi) - vy * sin(epsi)) / (1 - curvature * ey) * curvature);
    next_state(6, 0) = ey + deltaT * (vx * sin(epsi) + vy * cos(epsi));
    next_state(7, 0) = s + deltaT * (vx * cos(epsi) - vy * sin(epsi)) / (1 - curvature * ey);

    vx = next_state(0, 0);
    vy = next_state(1, 0);
    wz = next_state(2, 0);
    wF = next_state(3, 0);
    wR = next_state(4, 0);
    epsi = next_state(5, 0);
    ey = next_state(6, 0);
    s = next_state(7, 0);
    i++;
    //std::cout << "Pred Vx: " << next_state(0,0) << " Pred Vy: " << next_state(1, 0) << " Pred Wz: " << next_state(2, 0) << " Pred wF: " << next_state(3,0) << " Pred wR: " << next_state(4, 0) << " Pred epsi: " << next_state(5, 0) << " Pred ey: " << next_state(6, 0) << " iter: " << i << std::endl;
  }
  return next_state;
}

double VehicleDynamics::unnormalizeSteering(const double &steering)
{
  /* Unnormalize for input into vehicle dynamics model */
  return -25.0 * M_PI / 180.0 * steering;
}

double VehicleDynamics::unnormalizeThrottle(const double &throttle)
{
  /* Unnormalize for input into vehicle dynamics model */

  if (throttle > 0)
  {
    return 8.0 * throttle;
  }
  else
  {
    return 4.0 * throttle;
  }
}

void VehicleDynamics::LinearizeDynamics(const Eigen::Matrix<double, m_nx, 1> &state, const Eigen::Matrix<double, m_nu, 1> &input, const double &curvature)
{

    double delta = 0.000001;
    Eigen::Matrix<double, m_nx, 1> xplus, xminus, fplus, fminus;
    Eigen::Matrix<double, m_nu, 1> uplus, uminus;
    for(int i = 0; i < m_nx; ++i){
      xplus = state;
      xplus(i) += delta;
      xminus= state;
      xminus(i) -= delta;
      fplus = RWDBicycleModel(xplus, input, curvature);
      fminus = RWDBicycleModel(xminus, input, curvature);
      m_A.block(0, i, m_nx, 1) = 0.5/delta*( fplus - fminus );
    }
    for(int i = 0; i < m_nu; ++i){
      uplus = input;
      uplus(i) += delta;
      uminus= input;
      uminus(i) -= delta;
      fplus = RWDBicycleModel(state, uplus, curvature);
      fminus = RWDBicycleModel(state, uminus, curvature);
      m_B.block(0, i, m_nx, 1) = 0.5/delta*( fplus - fminus );

    }
    //std::cout <<"B: " <<m_B << std::endl;
    //std::cout << std::endl;
    m_d = RWDBicycleModel(state, input, curvature) - m_A * state - m_B * input;
}
