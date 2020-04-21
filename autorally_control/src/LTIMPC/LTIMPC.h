
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

// #include "../PID_CA/map.h" 
#include <VehicleDynamics/VehDyn.h>

#define _USE_MATH_DEFINES
namespace autorally_control
{
class LTIMPC
{
	private:
	    float m_s = 0.0;
        float m_n = 0.0;
        float m_epsi = 0.0;
        float m_yaw = 0.0;
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
        // bool m_useThetaGPS = false;
        // bool m_usePoseEstimate = false;
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

        ros::NodeHandle m_nh;
        // ros::Subscriber m_speedSub;
        // ros::Subscriber m_odomSub;
        // ros::Subscriber m_poseSub;
        ros::Subscriber m_mapCASub;
        ros::Publisher  m_chassisCommandPub; // Publsher for throttle commands
        ros::Publisher m_maskPub; // Publisher for steering angle commands
        ros::Publisher  m_runstopPub; // Publsher for throttle commands
        ros::Publisher m_trajectoryPub; // Publisher for MPC trajectory visualization
        
        //Vizualize trajectory
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_array; 

        // nav_msgs::Odometry m_position;
        // nav_msgs::Odometry m_prevPos;
        
        dynamic_reconfigure::Server<LTIMPC_paramsConfig> m_dynServer; 
        
        tf::TransformListener m_tf;
        boost::mutex m_lock;
            
        autorally_msgs::chassisCommand command;


        // Map map_instance;
        VehicleDynamics Vehicle;
        
        void LTIMPCcb();
        Eigen::Matrix<double, 8, 1> computeState();
        Eigen::Matrix<double, 2, 1> computeInput();
        void setMPCCost();
        // void WheelSpeedcb(autorally_msgs::wheelSpeeds speeds);// obtain front/rear wheel speeds 
        // void Gpscb(nav_msgs::Odometry position); // obtain s, n, and e_psi info from GPS signal
        void Solve(autorally_private_msgs::mapCA CA_states); // Solve the problem and return control command to ROS
        void ConfigCallback(const LTIMPC_paramsConfig &config, uint32_t level);
        void ViewMPCTrajectory(float state_est_x, float state_est_y, float state_est_yaw);

 	public:
     	LTIMPC();
    	~LTIMPC();
};
};
