
// This file tries to test the functions in LTIMPC.cpp

#include <iostream>
#include "eigen3/Eigen/Core"
#include "../PID_CA/map.h"

double unnormalizeSteering(double steering){
    return -25.0 * M_PI/180.0 * steering;
}

double unnormalizeThrottle(double throttle){
    if (throttle > 0){
      return 8.0 * throttle;
    } else {
      return 4.0 * throttle;
    }
}

double m_Vehicle_m = 23.91;
double m_Vehicle_Iz = 0.306744;
double m_Vehicle_lF = 0.34;
double m_Vehicle_lR = 0.23;
double m_Vehicle_IwF = 0.003759;
double m_Vehicle_IwR = 0.00408;
double m_Vehicle_rF = 0.095;
double m_Vehicle_rR = 0.095;
double m_Vehicle_mu1 = 0.75;
double m_Vehicle_mu2 = 0.7;

Eigen::Matrix<double, 8, 1> RWDBicycleModel(const Eigen::Matrix<double, 8, 1> &state, const Eigen::Matrix<double, 2, 1> &input){
    double deltaT = 0.001;

    double m_Vehicle_h = 0.2;// Need to check this value
    double m_g = 9.8065;
    double m_dt = 0.1;
    Eigen::Matrix<double, 8, 1> next_state(8, 1); 
    
    double delta = unnormalizeSteering( input(0,0) );
    double T = unnormalizeThrottle( input(1,0) ); 

    double vx = state(0,0);
    double vy = state(1,0);
    double wz = state(2,0);
    double wF = state(3,0);
    double wR = state(4,0);
    double epsi = state(5,0);
    double ey = state(6,0);
    double s = state(7,0);

    double cur; 

    double tire_B = 1.1559;
    double tire_C = 1.1924;
    double tire_D = 0.9956;
    double tire_E =-0.8505;
    double tire_Sh =-0.0540;
    double tire_Sv= 0.1444;

    Map map_instance;

    int i = 1;
    while( i * deltaT <= m_dt ){
      // Compute vertical forces on tire
      double fFz = m_Vehicle_m*m_g * ( m_Vehicle_lR-m_Vehicle_h*m_Vehicle_mu1) / (m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h*(m_Vehicle_mu1*cos(delta) - m_Vehicle_mu2*sin(delta) - m_Vehicle_mu1));
      double fRz = m_Vehicle_m*m_g - fFz ;
      // Theoretical slip quantities
      double beta;
      if (vx < 0.2){
          vx = 0.2;
      }
      if(vx != 0.0){
          beta = atan2(vy,vx);
      }else{
          beta = 0.0;
      }
      double V = sqrt(vx*vx + vy*vy);
      double vFx = V*cos(beta-delta) + wz*m_Vehicle_lF*sin(delta);
      double vFy = V*sin(beta-delta) + wz*m_Vehicle_lF*cos(delta);
      double vRx = vx;
      double vRy = vy - wz*m_Vehicle_lR;
      double sFx, sRx;
      if (wF != 0.0){
        sFx = (vFx - wF*m_Vehicle_rF)/(wF*m_Vehicle_rF);
        // std::cout <<"sFx: " << sFx << std::endl;
      } else {
        sFx = 0.0;
      }
      if (wR != 0.0){
        sRx = (vRx - wR*m_Vehicle_rR)/(wR*m_Vehicle_rR);
        // std::cout <<"sRx: " << sRx << std::endl;
      } else {
        sRx = 0.0;
      }
      double sFy, sRy;
      if(vFx != 0.0){
        sFy = (1 + sFx)*vFy/vFx;
      } else {
        sFy = 0.0;
      }
      if(vRx != 0.0){
        sRy = (1 + sRx)*vRy/vRx;
      } else {
        sRy = 0.0;
      }
      double sF = sqrt(sFx*sFx + sFy*sFy);
      double sR = sqrt(sRx*sRx + sRy*sRy);

      double sEF = sF - tire_Sh;
      double sER = sR - tire_Sh;

    //   double muF = tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(sEF) ) ) ) + tire_Sv; 
    //   double muR = tire_D*sin( tire_C*atan( tire_B*sER - tire_E*(tire_B*sER - atan(sER) ) ) ) + tire_Sv;
      double muF = tire_D*sin( tire_C*atan( tire_B*sF ) ); 
      double muR = tire_D*sin( tire_C*atan( tire_B*sR ) );

      double fFx = -sFx/sF * muF * fFz;
      double fFy = -sFy/sF * muF * fFz;
      double fRx = -sRx/sR * muR * fRz;
      double fRy = -sRy/sR * muR * fRz;
      // Obtain current road curvature
      cur = map_instance.obtainCurvature(s); 
    //   std::cout << "sF and sR: " << sF << ", " << sR << std::endl;
    //   std::cout << "fFx: " << fFx << std::endl;

      // compute next state using (V.6)
      next_state(0,0) = vx + deltaT*( (fFx*cos(delta) - fFy*sin(delta) + fRx)/m_Vehicle_m + vy*wz);
      next_state(1,0) = vy + deltaT*( (fFx*sin(delta) + fFy*cos(delta) + fRy)/m_Vehicle_m - vx*wz);
      next_state(2,0) = wz + deltaT*( (fFy*cos(delta) + fFx*sin(delta))*m_Vehicle_lF - fRy*m_Vehicle_lR )/m_Vehicle_Iz;
      next_state(3,0) = wF - deltaT*m_Vehicle_rF/m_Vehicle_IwF*fFx;
      next_state(4,0) = wR + deltaT/m_Vehicle_IwR*( T - m_Vehicle_rR*fRx );
      next_state(5,0) = epsi + deltaT*(wz - ( vx*cos(epsi) - vy*sin(epsi) )/( 1-cur*ey )*cur);
      next_state(6,0) = ey + deltaT*( vx*sin(epsi) + vy*cos(epsi) );
      next_state(7,0) = s + deltaT*(vx*cos(epsi) - vy*sin(epsi))/(1 - cur*ey);

      vx = next_state(0,0);
      vy = next_state(1,0);
      wz = next_state(2,0);
      wF = next_state(3,0);
      wR = next_state(4,0);
      epsi = next_state(5,0);
      ey = next_state(6,0);
      s = next_state(7,0);
    //   std::cout << fFx << ", " << fFy << ", " << fRx << ", " << fRy << std::endl;
    //   std::cout << "next State: " << std::endl;
    //   std::cout << next_state << std::endl;
      
      i++;
    }
    return next_state; 
  }
void LinearizeDynamics(const Eigen::Matrix<double, 8, 1> &state, const Eigen::Matrix<double, 2, 1> &input){
    float delta = 0.0001;
    Eigen::Matrix<double, 8, 1> xplus, xminus, fplus, fminus;
    Eigen::Matrix<double, 2, 1> uplus, uminus;
    Eigen::Matrix<double, 8, 8, Eigen::ColMajor> m_A; 
    Eigen::Matrix<double, 8, 2, Eigen::ColMajor> m_B;
    Eigen::Matrix<double, 8, 1> m_d;
    int m_nx = 8;
    int m_nu = 2;
    for(int i = 0; i < m_nx; ++i){
      for(int j = 0; j < m_nx; ++j){
        m_A(i,j) = 0.0;
      }
    }

    for(int i = 0; i < m_nx; ++i){
      for(int j = 0; j < m_nu; ++j){
        m_B(i,j) = 0.0;
      }
    }

    for(int i = 0; i < m_nx; ++i){
      m_d(i) = 0.0;
    }

    for(int i = 0; i < m_nx; ++i){
      xplus = state;
      xplus(i) += delta;
      xminus= state;
    //   xminus(i) -= delta;
      fplus = RWDBicycleModel(xplus, input);
      fminus = RWDBicycleModel(xminus, input);
      m_A.block(0, i, m_nx, 1) = 0.5/delta*( fplus - fminus );
      std::cout << "A: " <<  m_A << std::endl;
    }
    for(int i = 0; i < m_nu; ++i){
      uplus = input;
      uplus(i) += delta;
      uminus= input;
    //   uminus(i) -= delta;
      fplus = RWDBicycleModel(state, uplus);
      fminus = RWDBicycleModel(state, uminus);
      m_B.block(0, i, m_nx, 1) = 0.5/delta*( fplus - fminus );
    }
    m_d = RWDBicycleModel(state, input) - m_A * state - m_B * input;
    std::cout << "A: " << m_A << std::endl;
    std::cout << "B: " << m_B << std::endl;
    std::cout << "d: " << m_d << std::endl;
  }

int main (int argc, char** argv)
{
  Eigen::Matrix<double, 8, 1> state;
  Eigen::Matrix<double, 2, 1> input;
  double nom_vel = 0.15;
  state(0,0) = nom_vel;
  state(1,0) = 0.0;
  state(2,0) = 0.0;
  state(3,0) = 0.9*nom_vel/m_Vehicle_rF;
  state(4,0) = 1.1*nom_vel/m_Vehicle_rR;
  state(5,0) = 0.0;
  state(6,0) = 0.0;
  state(7,0) = 0.0;

  input(0,0) = 0.0;
  input(1,0) = 0.0;
  
//   std::cout << RWDBicycleModel(state, input) << std::endl;
  LinearizeDynamics(state, input);


}