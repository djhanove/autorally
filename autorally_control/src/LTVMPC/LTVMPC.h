
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <autorally_msgs/chassisCommand.h>
#include <autorally_msgs/mapCA.h>
#include <autorally_control/LTVMPC_paramsConfig.h> //point to cfg file, catkin will generate the header 
#include <boost/thread.hpp>          // Mutex
#include <boost/lexical_cast.hpp>
#include <future>
#include <thread>
#include <autorally_control/VehicleDynamics/VehDyn.h>
#include <autorally_control/Utilities/Utilities.h>

#define _USE_MATH_DEFINES
namespace autorally_control
{
class LTVMPC
{
	private:
        autorally_msgs::chassisCommand command;
        boost::mutex m_lock;

        double speedCommand = 0.0;
        const double m_dt = 0.001; 

        double time;
        double dt;
        double prevTime = 0.0;
        double curvature;
        const double controllerUpdateRate = 0.01;

        
        dynamic_reconfigure::Server<LTVMPC_paramsConfig> m_dynServer; 

        Eigen::Matrix<double, 8, 8, Eigen::ColMajor> m_Q;
        Eigen::Matrix<double, 2, 2, Eigen::ColMajor> m_R;
        Eigen::Matrix<double, 8, 1> x0;
        Eigen::Matrix<double, 2, 1> control;
        Eigen::Matrix<double, 13, 8> m_linPoints;


        int m_N = 12; // Horizon
        int m_nX = 8; // Number of states
        int m_nU = 2; // Number of control outputs

        ros::NodeHandle nh;
        ros::Subscriber mapCASub;
        ros::Publisher  chassisCommandPub; // Publsher for throttle commands
        ros::Publisher maskPub; // Publisher for steering angle commands
        ros::Publisher  runstopPub; // Publsher for throttle commands

        nav_msgs::Odometry m_position;
        nav_msgs::Odometry m_prevPos;
        
        tf::TransformListener m_tf;
            
        Utilities util;

        VehDyn Vehicle;
        
        void LTVMPCcb();
        void setMPCCost();
        void Solve(autorally_msgs::mapCA CA_states);
        void ConfigCallback(const LTVMPC_paramsConfig &config, uint32_t level);
        void populateParameterArrays(double (&A)[64], double (&B)[16], double (&d)[8],
            const Eigen::Matrix<double, 8, 8, Eigen::ColMajor> &eigenA, const Eigen::Matrix<double, 8, 2, 
            Eigen::ColMajor> &eigenB, const Eigen::Matrix<double, 8, 1> &eigend);
        void asyncMatrixPopulation(int k);

 	public:
     	LTVMPC(std::string prefix);
};
}; // ns
