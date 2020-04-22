#include <eigen3/Eigen/Dense>
#include <boost/thread.hpp>          // Mutex
#include <boost/lexical_cast.hpp>

class VehDyn
{
private:
    // ** Vehicle Parameters ** //
    double Vehicle_m = 21.88; // total mass of vehicle (kg)
    double Vehicle_Iz = 1.124; // Yaw-moment of inertia (kg-m^2);
    double Vehicle_lF = 0.34; // Length from CG to front axle(m)
    double Vehicle_lR = 0.23; // Length from CG to rear axle (m)
    double Vehicle_IwF = 0.048;// Rolling moment of inertia of front wheels (kg-m^2)
    double Vehicle_IwR = 0.044;// Rolling moment of inertia of rear wheels (kg-m^2)
    double Vehicle_rF = 0.095; // Radius of Front Wheel (m)
    double Vehicle_rR = 0.095; // Radius of Rear Wheel (m)
    double Vehicle_mu1 = 0.7;  // Gazebo world parameter http://gazebosim.org/tutorials?tut=physics_params&cat=physics
    double Vehicle_mu2 = 0.75; // Gazebo world parameter http://gazebosim.org/tutorials?tut=physics_params&cat=physics
    double Vehicle_h = 0.2;    // Height of CG (m)
    double accel_g = 9.80665; // gravitational constant (m/s^2)
    
    // ** Tire Parameters for Magic Formula (Pacjeka) ** //
    double tire_B = 1.1559;
    double tire_C = 1.1924;
    double tire_D = 0.9956;
    double tire_E = -0.8505;
    double tire_Sh = -0.0540;
    double tire_Sv = 0.1444;
    
    // ** State Propagation (time parameters) ** //
    double deltaT = 0.001; // dT increment for state propagation (s)
    double timeStep = 0.1; // length of time for each prediction step in the horizon (s)

    double unnormalizeSteering(const double &ctrlSteer);
    double unnormalizeThrottle(const double &ctrlThrottle);  
    Eigen::Matrix<double, 8, 1> RWDBicycleModel(const Eigen::Matrix<double, 8, 1> &state, const Eigen::Matrix<double, 2, 1> &control, const double &curvature); 
    double calcSideSlipAngle(double &vx, const double &v);
    double calcSlipRatio(double const &velLong, double const &omega);
    double calcSlipAngle(double const &velLong, double const &velLat, double const &slipRatio);

public:
    VehDyn();
    double getFrontWheelRadius() {return Vehicle_rF;}
    double getRearWheelRadius() {return Vehicle_rR;}
    void LinearizeDynamics(const Eigen::Matrix<double, 8, 1> &, const Eigen::Matrix<double, 2, 1>&,  const double &curvature);
    // ** Eigen Matrices for finding linearized dynamics ** //
    Eigen::Matrix<double, 8, 8, Eigen::ColMajor> m_A; 
    Eigen::Matrix<double, 8, 2, Eigen::ColMajor> m_B;
    Eigen::Matrix<double, 8, 1> m_d;
};

