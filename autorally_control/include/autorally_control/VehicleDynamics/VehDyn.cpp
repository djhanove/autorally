#include "VehDyn.h"

VehDyn::VehDyn() {
    m_A.setZero();
    m_B.setZero();
    m_d.setZero();
}

Eigen::Matrix<double, 8, 1> VehDyn::RWDBicycleModel(const Eigen::Matrix<double, 8, 1> &state, const Eigen::Matrix<double, 2, 1> &control, const double &curvature)
{
    /* Bicycle model with Pacejka tire model. Protects are included for when vehicle speed is near zero, as Pacejka models tend
        to output incorrect slip angle and force predictions when vel is near zero */

    double delta = unnormalizeSteering(control(0, 0)); // Road wheel angle (rad)
    double T = unnormalizeThrottle(control(1, 0)); // Wheel Torque
    double velX = state(0, 0); // x (longitudinal) velocity (m/s) at CG
    double velY = state(1, 0); // y (lateral) velocity (m/s) at CG
    double omegaZ = state(2, 0); // yaw rate, omega z, (rad/s) at CG
    double omegaFront = state(3, 0); // front wheel angular velocity, omega front (rad/s)
    double omegaRear = state(4, 0); // rear wheel angular velocity, omega rear (rad/s)
    double errorPsi = state(5, 0); // heading deviation (error) (rad)
    double errorY = state(6, 0); // lateral deviation from path  (error) (m)
    double s = state(7, 0); // distance traveled along path (m)
    Eigen::Matrix<double, 8, 1> next_state;
    
    // Loop through vehicle dynamics model with a small dt many times for next vehicle state
    for (size_t i = 0; i < int(timeStep / deltaT); i++)
    {
        // Compute vertical forces on tire
        double forceFz = Vehicle_m * accel_g * (Vehicle_lR - Vehicle_h * Vehicle_mu1) / (Vehicle_lF + Vehicle_lR 
                    + Vehicle_h * (Vehicle_mu1 * cos(delta) - Vehicle_mu2 * sin(delta) - Vehicle_mu1));
        double forceRz = Vehicle_m * accel_g - forceFz;

        // Theoretical slip quantities
        double beta = calcSideSlipAngle(velX, velY);

        double velVector = sqrt(velX * velX + velY * velY); // Get velocity vector
        double velFx = velVector * cos(beta - delta) + omegaZ * Vehicle_lF * sin(delta);
        double velFy = velVector * sin(beta - delta) + omegaZ * Vehicle_lF * cos(delta);
        double velRx = velX;
        double velRy = velY - omegaZ * Vehicle_lR;
        double slipRatioFront = calcSlipRatio(velFx, omegaFront);
        double slipRatioRear = calcSlipRatio(velRx, omegaRear);
        double slipAngleFront = calcSlipAngle(velFx, velFy, slipRatioFront);
        double slipAngleRear = calcSlipAngle(velRx, velRy, slipRatioRear);

        // Get slip vector for front and rear tires 
        double sF = sqrt(slipAngleFront * slipAngleFront + slipRatioFront * slipRatioFront); 
        double sR = sqrt(slipAngleRear * slipAngleRear + slipRatioRear * slipRatioRear); 

        // Calculate friction value for front and rear tires using reduced magic formula
        double muF = tire_D * sin(tire_C * atan(tire_B * sF));
        double muR = tire_D * sin(tire_C * atan(tire_B * sR));
        
        // Compute Longitudinal and Lateral forces at front/rear tires
        double fFx = -slipRatioFront / sF * muF * forceFz;
        double fFy = -slipAngleFront / sF * muF * forceFz;
        double fRx = -slipRatioRear / sR * muR * forceRz;
        double fRy = -slipAngleRear / sR * muR * forceRz;

        // compute next state using (V.6)
        velX += deltaT * ((fFx * cos(delta) - fFy * sin(delta) + fRx) / Vehicle_m + velY * omegaZ);
        velY += deltaT * ((fFx * sin(delta) + fFy * cos(delta) + fRy) / Vehicle_m - velX * omegaZ);
        omegaZ += deltaT * ((fFy * cos(delta) + fFx * sin(delta)) * Vehicle_lF - fRy * Vehicle_lR)
             / Vehicle_Iz;
        omegaFront -= deltaT * Vehicle_rF / Vehicle_IwF * fFx;
        omegaRear += deltaT / Vehicle_IwR * (T - Vehicle_rR * fRx);
        errorPsi += deltaT * (omegaZ - (velX * cos(errorPsi) - velY * sin(errorPsi)) / (1 - curvature * errorY) * curvature);
        errorY += deltaT * (velX * sin(errorPsi) + velY * cos(errorPsi));
        s += deltaT * (velX * cos(errorPsi) - velY * sin(errorPsi)) / (1 - curvature * errorY);
    }
    next_state(0,0) = velX;
    next_state(1,0) = velY;
    next_state(2,0) = omegaZ;
    next_state(3,0) = omegaFront;
    next_state(4,0) = omegaRear;
    next_state(5,0) = errorPsi;
    next_state(6,0) = errorY;
    next_state(7,0) = s;
    return next_state;
} 

double VehDyn::unnormalizeSteering(const double &ctrlSteer)
{
    /* 
       Takes in a normalized steering command from -1.0 -> 1.0 and converts
       to a road wheel angle, delta, in radians for the dynamics model
    */
    return -25.0 * M_PI / 180.0 * ctrlSteer;
}
double VehDyn::unnormalizeThrottle(const double &ctrlThrottle) 
{
    /* 
       Maps throttle command to a wheel torque via scalar multiplier per AutoRally spec
       Returns wheel torque (N-m)
    */
   return (ctrlThrottle > 0) ? (8.0 * ctrlThrottle) : (4.0 * ctrlThrottle);
}  

double VehDyn::calcSideSlipAngle(double &velX, const double &velY)
{
    /* Calculates the vehicle side slip angle via longitudinal and
       lateral velocities. Need to protect for when longitudinal velocity
       is low because the Pac tire model goes unstable
       See: https://www.edy.es/dev/2011/12/facts-and-myths-on-the-pacejka-curves/
    */
    if (velX < 0.2)
        velX = 0.2;
    return atan2(velY, velX);
}

double VehDyn::calcSlipRatio(double const &velLong, double const &omega)
{
    /* Finds the slip ratio for a given wheel. Protects for the case when angular velocity is zero
       Formula referenced from TU Delft's magic formula 6.2
    */
    return (omega > 0.0) ? (velLong - omega * Vehicle_rF) / (omega * Vehicle_rF) : 0.0;
}

double VehDyn::calcSlipAngle(double const &velLong, double const &velLat, double const &slipRatio)
{    
    /* Finds the slip angle for a given wheel. Protects for the case when longitudinal velocity is zero
       Formula referenced from TU Delft's magic formula 6.2
    */
    return (velLong > 0.0) ? ((1 + slipRatio) * velLat / velLong) : 0.0;
}

void VehDyn::LinearizeDynamics(const Eigen::Matrix<double, 8, 1> &state, const Eigen::Matrix<double, 2, 1>&control,  const double &curvature)
{
    double perturb = 0.000001; // arbitrary perturbation across all state and control values
    Eigen::Matrix<double, 8, 1> xplus, xminus, fplus, fminus;
    Eigen::Matrix<double, 2, 1> uplus, uminus;
    // copy values from state and control over
    xplus = state;
    xminus = state;
    uplus = control;
    uplus = control;

    // Perturb state variables on both sides and find localized linear model for State Matrix A
    for(int i = 0; i < 8; ++i){
      xplus(i) += perturb;
      xminus(i) -= perturb;
      fplus = RWDBicycleModel(xplus, control, curvature);
      fminus = RWDBicycleModel(xminus, control, curvature);
      m_A.block(0, i, 8, 1) = 0.5 / perturb * ( fplus - fminus );
    }

    // Perturb control variables on both sides and find localized linear model for Control Matrix B
    for(int i = 0; i < 2; ++i){
      uplus(i) += perturb;
      uminus(i) -= perturb;
      fplus = RWDBicycleModel(state, uplus, curvature);
      fminus = RWDBicycleModel(state, uminus, curvature);
      m_B.block(0, i, 8, 1) = 0.5 / perturb * ( fplus - fminus );

    }
    // calculate disturbance matrix, d, using the newly linearized state and control matrices
    m_d = RWDBicycleModel(state, control, curvature) - m_A * state - m_B * control;
}