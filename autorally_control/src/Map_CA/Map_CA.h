#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/Odometry.h>


#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>

#include <autorally_msgs/chassisCommand.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/runstop.h>
#include <autorally_msgs/mapCA.h>
#include <autorally_msgs/lapStats.h>

#include <boost/thread.hpp>          // Mutex
#include <boost/lexical_cast.hpp>


namespace autorally_control
{
	class Map_CA
	{
 		private:

	 	ros::NodeHandle nh_;
	    ros::Subscriber odom_sub_;
	    ros::Subscriber speed_sub_;
	    ros::Publisher map_CA_pub_;
	    ros::Publisher lap_stats_pub_;

	    nav_msgs::Odometry curr_position_;
        nav_msgs::Odometry prev_position_;
        autorally_msgs::mapCA CA_states_;
        autorally_msgs::lapStats lap_stats_;

        tf::TransformListener tf_;
		tf::TransformBroadcaster odom_broadcaster;
		ros::Time current_time, last_time;


        boost::mutex lock_;
	    
	    bool use_pose_estimate_;
	 	
	 	// Map Parameters: x (Global X coordinate), y (Global X coordinate), psi (Yaw Angle), s (Distance along Centerline Covered), ls (Length of Segment), rho (Curvature of Segment)
	 	
	 	double map_parameters_[2][5][6] = 
	 	{
	 		{
				{-8.62, 8.38, 2.36, 0, 5.9821, 0},					 
				{-16.94, -0.19, -0.80, 5.9821, 18.7621, 0.1674}, 
				{-8.81, -8.64, -0.80, 24.7552, 11.726, 0}, 
				{-0.12, 0.05, 2.36, 36.4702, 19.304, 0.1627}, 
				{-4.37, 4.17, 2.36, 55.774, 5.919, 0}
			},

			{
				{2.78,-2.97,-0.6613, 0, 3.8022, 0},
				{10.04,6.19, 2.4829, 3.8022, 18.3537,0.1712 }, 
				{1.46, 13.11,2.4829, 22.1559, 11.0228 , 0},
	 			{-5.92, 3.80, -0.6613, 33.1787 , 18.6666, 0.1683},
				{-0.24, -0.66,-0.6613, 51.8453, 7.2218, 0}
			}
	 	};


	 	double s_bias_;			// Bias added to ensure that the s value is 0 from the point the controller starts indicating the start of the track	
	 	double s_prev_;			// Previous s value
	 	double start_time_;		// Start time of the controller
	 	double lap_start_time_;	// Start time of current lap
	 	double lap_max_speed_;	// Maximum speed of the current lap
	 	double elapsed_time_;	// Total Time since the start of the controller
	 	double lap_time_;		// Completed Lap Time
	 	double track_length_;	// Length of the Track
	 	double vx_;				// Longitudinal velocity in vehicle frame
	 	double vy_;				// Lateral velocity in vehicle frame
	 	double wz_;				// Yaw rate
	 	double wf_;				// Front wheel speed
	 	double wr_;				// Rear wheel speed
	 	double s_;				// Distance traveled along path
	 	double n_;				// Lateral deviation from center line, if centerline is to the left of you, n is negative
	 	double d_psi_;			// Heading direction deviation
	 	double curvature_;		// Curvature of the segment
	 	double speed_;			// Speed of the car. Calculated by taking the resultant magnitude of components vx and vy 

		int number_of_segments_;	// The number of regions of circular arcs and straight lines the track is divided into 
		int map_flag_;				// Indicates which map is being used
	 	int completed_laps_;		
	 	int iterations_;			
	 	int current_region_;		

		int indentifyRegion(double, double);
		
		std::string track_name_[4] = {"Marietta Gazebo", "Marietta Track", "CCRF Gazebo", "CCRF Track"};

		void GPSCallback(nav_msgs::Odometry);
		void wheelSpeedCallback(autorally_msgs::wheelSpeeds);
        void calculateStraightSegment(int, int, double, double, double);
	 	void calculateCurvedSegment(int, int, double, double, double);
	 
	 	public:
	 		
	 	Map_CA();
	 	~Map_CA();


	};
};



