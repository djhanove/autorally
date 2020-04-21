
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>

#include <autorally_msgs/chassisCommand.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/runstop.h>
#include <autorally_private_msgs/mapCA.h>


#include <autorally_private_control/LTVMPC_paramsConfig.h> //point to cfg file, catkin will generate the header 

#include <boost/thread.hpp>          // Mutex
#include <boost/lexical_cast.hpp>

#include <VehicleDynamics/vehicle_dynamics.h>
#include <Utilities/Utilities.h>

#define _USE_MATH_DEFINES
namespace autorally_control
{
class LTVMPC
{
	private:
	    float m_s;
        float m_n;
        float m_epsi;
        float m_yaw;
        float m_point[2] = {}; 
        
        float m_iterations = 0;

        float m_speedCommand = 0.0;
        float m_frontWheelsSpeed = 0.0;
		float m_rearWheelsSpeed = 0.0;
        float m_dt = 0.001; 

        double time;
        double dt;

        double m_speed;
        double m_wpRadius;
        double m_headingP;
        double m_offsetX, m_offsetY;
        double m_prevTime;
        double m_currTime; 

        double m_vx;
        double m_vy;
        double m_wz;


        double cur;



        // Cost Function
        Eigen::Matrix<double, 8, 8, Eigen::ColMajor> m_Q;
        Eigen::Matrix<double, 2, 2, Eigen::ColMajor> m_R;
        int m_N = 12; // Horizon

        double m_linPoints[13][8];

        ros::NodeHandle m_nh;
        ros::Subscriber m_mapCASub;
        ros::Publisher  m_chassisCommandPub; // Publsher for throttle commands
        ros::Publisher m_maskPub; // Publisher for steering angle commands
        ros::Publisher  m_runstopPub; // Publsher for throttle commands

        nav_msgs::Odometry m_position;
        nav_msgs::Odometry m_prevPos;
        
        dynamic_reconfigure::Server<LTVMPC_paramsConfig> m_dynServer; 
        
        tf::TransformListener m_tf;
        boost::mutex m_lock;
            
        autorally_msgs::chassisCommand command;

        Utilities util;

        VehicleDynamics Vehicle;
        
        void LTVMPCcb();
        Eigen::Matrix<double, 8, 1> computeState();
        Eigen::Matrix<double, 2, 1> computeInput();
        void setMPCCost();
        void Solve(autorally_private_msgs::mapCA CA_states); // Solve the problem and return control command to ROS
        void ConfigCallback(const LTVMPC_paramsConfig &config, uint32_t level);
        void populateParameterArrays(double (&A)[64], double (&B)[16], double (&d)[8]);

 	public:
     	LTVMPC();
    	~LTVMPC();
};
};
