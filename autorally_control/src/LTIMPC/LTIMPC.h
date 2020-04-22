
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>

#include <autorally_msgs/chassisCommand.h>
// #include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/runstop.h>
#include <autorally_msgs/mapCA.h>

#include <autorally_control/LTIMPC_paramsConfig.h> //point to cfg file, catkin will generate the header 

#include <boost/thread.hpp>          // Mutex
#include <boost/lexical_cast.hpp>

#include "VehDyn.h"

#define _USE_MATH_DEFINES
namespace autorally_control
{
class LTIMPC
{
	private:
        autorally_msgs::chassisCommand command;
        
        boost::mutex m_lock;

        double speedCommand = 0.0;
        double m_dt = 0.001;
        double time;
        double dt;
        double prevTime = 0.0;
        double curvature;
        double controllerUpdateRate = 0.01;

        double *x_out_ptr; 
        double *A_ptr;
        double *B_ptr;
        double *d_ptr;
        double *Q_ptr;
        double *R_ptr;

        dynamic_reconfigure::Server<LTIMPC_paramsConfig> m_dynServer; 
        dynamic_reconfigure::Server<LTIMPC_paramsConfig>::CallbackType cb;

        Eigen::Matrix<double, 8, 8, Eigen::ColMajor> m_Q;
        Eigen::Matrix<double, 2, 2, Eigen::ColMajor> m_R;
        Eigen::Matrix<double, 8, 1> x0;
        Eigen::Matrix<double, 2, 1> control;

        ros::NodeHandle nh;
        ros::Subscriber mapCASub; // Subscriber for Map topics
        ros::Publisher  chassisCommandPub; // Publsher for throttle commands
        ros::Publisher maskPub; // Publisher for steering angle commands
        ros::Publisher  runstopPub; // Publsher for runstop commands
        ros::Publisher trajectoryPub; // Publisher for MPC trajectory visualization
        
        tf::TransformListener m_tf;

        VehDyn Vehicle;

        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_array; 

        void LTIMPCcb();
        void setMPCCost();
        void Solve(const autorally_msgs::mapCA &CA_states); // Solve the problem and return control command to ROS
        void ConfigCallback(const LTIMPC_paramsConfig &config, uint32_t level);
        void getPointerstoEigen();
        //void ViewMPCTrajectory(float state_est_x, float state_est_y, float state_est_yaw);

 	public:
     	LTIMPC();
    	~LTIMPC(){};


};
};
