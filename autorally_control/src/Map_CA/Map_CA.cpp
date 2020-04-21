#include "Map_CA.h"

namespace autorally_control
{
	Map_CA::Map_CA(): nh_("~"), map_flag_(0), wf_(0.0), wr_(0.0), s_prev_(0.0), iterations_(0), completed_laps_(0), s_bias_(0), current_region_(0), use_pose_estimate_(false)
	{
		// Get Parameters from launch file
		// @params:
		// map_flag_(int): (0: Marietta Gazebo, 1: Marietta Track, 2: CCRF Gazebo, 3: CCRF Track)
		// use_pose_estimate(bool): (true: use pose estimate from state estimator, false: use ground truth in simulation)
		nh_.getParam("MAP_CA/map_flag", map_flag_);
		nh_.getParam("MAP_CA/use_pose_estimate", use_pose_estimate_);
		// std::cout << "Using Pose Estimate:" << use_pose_estimate_ << std::endl;
		// std::cout << "Map Flag:" << n << std::endl;
		speed_sub_  = nh_.subscribe("/wheelSpeeds", 1, &Map_CA::wheelSpeedCallback, this);
		if (use_pose_estimate_)
			odom_sub_   = nh_.subscribe("/pose_estimate", 1, &Map_CA::GPSCallback, this);
	    else
			odom_sub_   = nh_.subscribe("/ground_truth/state", 1, &Map_CA::GPSCallback, this);
			current_time = ros::Time::now();
  			last_time = ros::Time::now();
	    map_CA_pub_ = nh_.advertise<autorally_private_msgs::mapCA>("mapCA", 1);	            
		lap_stats_pub_ = nh_.advertise<autorally_msgs::lapStats>("lapStats", 1);	            
		
		// Marietta Track is divided into 5 regions of circular arcs and straight segments, CCRF is divided into 15 such segments
		if (map_flag_ == 0 || map_flag_ == 1)
			number_of_segments_ = 5;
		else
			number_of_segments_ = 15;
		track_length_ = map_parameters_[map_flag_][number_of_segments_-1][3] + map_parameters_[map_flag_][number_of_segments_-1][4];
		ros::Duration d = ros::Duration(1.0);
        d.sleep();

	}

	Map_CA::~Map_CA()
		{}

	void Map_CA::GPSCallback(nav_msgs::Odometry odom)
	{
	    double x, y, theta;
	    lock_.lock();
	    curr_position_ = odom;

	    x = curr_position_.pose.pose.position.x;
	    y = curr_position_.pose.pose.position.y;

	    // Get yaw from the message
	    double roll, pitch, yaw;
	    tf::Quaternion quat(curr_position_.pose.pose.orientation.x,
	                        curr_position_.pose.pose.orientation.y,
	                        curr_position_.pose.pose.orientation.z,
	                        curr_position_.pose.pose.orientation.w);
	    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	    double delta_X = x - prev_position_.pose.pose.position.x;
	    double delta_Y = y - prev_position_.pose.pose.position.y;
	    lock_.unlock();

	    current_region_ = indentifyRegion(x, y);
		int prev_reg = current_region_ - 1;
		if (prev_reg == -1)
			prev_reg = number_of_segments_ - 1;

		curvature_ = map_parameters_[map_flag_][current_region_][5];

		// Calculate the s, ey, epsi from x, y, yaw depending on whether the region is a straight segment or a circular arc
		if (curvature_ == 0)
			calculateStraightSegment(current_region_, prev_reg, x, y, yaw);
		else
			calculateCurvedSegment(current_region_, prev_reg, x, y, yaw);

		
		vx_ = curr_position_.twist.twist.linear.x * cos(yaw) + curr_position_.twist.twist.linear.y * sin(yaw);  //transforms to robot frame
 		vy_ = curr_position_.twist.twist.linear.x * -sin(yaw) + curr_position_.twist.twist.linear.y * cos(yaw); //transforms to robot frame
  		wz_ = curr_position_.twist.twist.angular.z;

        double path_s;
        speed_ = sqrt(vx_*vx_ + vy_*vy_);
        
        // Get the bias on s to ensure that when the controller starts, the s value is 0 indicating the start of the track
        if (iterations_ == 0)
        {
        	s_bias_ = s_;
        	start_time_ = ros::Time::now().toSec();
        	lap_start_time_ = ros::Time::now().toSec();
        	lap_max_speed_ = 0;	
        }

        // Get the maximum lap speed
        if (speed_ > lap_max_speed_)
        	lap_max_speed_ = speed_;
        
       	path_s = s_ - s_bias_;
       	
       	if (path_s < 0)
        		path_s += track_length_;

        // Publish the lap stats message with lap time, lap maximum speed and current lap	
        if (s_prev_ > (track_length_ - 1.0) && path_s < 1.0 && lap_time_ > 5.0) // Condition for lap time > 5.0 is to ensure that when the vehicle oscillates at starting point, it does not count as multiple laps (Could be set to any realistic value)  
        {
        	completed_laps_++;
        	lap_start_time_ = ros::Time::now().toSec();
        	lap_stats_.lap_number = completed_laps_;
        	lap_stats_.lap_time = lap_time_;
        	lap_stats_.max_speed = lap_max_speed_;
        	lap_stats_pub_.publish(lap_stats_);
        	lap_max_speed_ = 0;
      	}  
       	
       	
        elapsed_time_ = ros::Time::now().toSec() - start_time_;
        lap_time_ = ros::Time::now().toSec() - lap_start_time_;

		CA_states_.header.stamp = ros::Time::now();
		CA_states_.track_length = track_length_;
		CA_states_.track_name = track_name_[map_flag_];
		CA_states_.vx = vx_;
        CA_states_.vy = vy_;
        CA_states_.wz = wz_;
        CA_states_.wf = wf_;
        CA_states_.wr = wr_;
        CA_states_.s = s_;//You can also publish path_s to better visualize the result. 
        CA_states_.ey = n_;
        CA_states_.epsi = d_psi_;
        CA_states_.curvature = curvature_;
        CA_states_.time = elapsed_time_;
        CA_states_.lap = completed_laps_;
        CA_states_.track_region = current_region_;
		CA_states_.x = x;
		CA_states_.y = y;
		CA_states_.yaw = yaw;


        // Publish Map_CA with the state of the system
        map_CA_pub_.publish(CA_states_);

	if (~use_pose_estimate_){
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
		//publish the ground truth transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		last_time = current_time;
	}
        
        s_prev_ = path_s;
        iterations_++;
	}


	void Map_CA::calculateStraightSegment(int curr_reg, int prev_reg, double x, double y, double psi)
	{
		double d_rel = sqrt( pow(x - map_parameters_[map_flag_][prev_reg][0], 2)  + pow (y - map_parameters_[map_flag_][prev_reg][1], 2) );
		double theta_rel = atan2(y - map_parameters_[map_flag_][prev_reg][1], x - map_parameters_[map_flag_][prev_reg][0] ) - 
						   atan2(map_parameters_[map_flag_][curr_reg][1] - map_parameters_[map_flag_][prev_reg][1], map_parameters_[map_flag_][curr_reg][0] - map_parameters_[map_flag_][prev_reg][0]);

	    if (theta_rel < -M_PI)
	    	theta_rel += 2.0 * M_PI;
	    else if (theta_rel > M_PI)
	    	theta_rel -= 2.0 * M_PI;

	    s_ = map_parameters_[map_flag_][curr_reg][3] + d_rel * cos(theta_rel);
	    n_ = d_rel * sin(theta_rel);
	    d_psi_ = psi - map_parameters_[map_flag_][curr_reg][2];

	    if (d_psi_ < -M_PI){
	    	while (d_psi_ < - M_PI){
	    		d_psi_ += 2.0 * M_PI;

	    	}

	    }
	    else if (d_psi_ > M_PI)
	    {
	    	while (d_psi_ > M_PI)
	    		d_psi_ -= 2.0 * M_PI;
	    }

	}


	void Map_CA::calculateCurvedSegment(int curr_reg, int prev_reg, double x, double y, double psi)
	{
		double x_c = 0.5 * (map_parameters_[map_flag_][curr_reg][0] + map_parameters_[map_flag_][prev_reg][0]);
	    double y_c = 0.5 * (map_parameters_[map_flag_][curr_reg][1] + map_parameters_[map_flag_][prev_reg][1]);
		
		double d_rel = sqrt( pow(x - x_c, 2) + pow(y - y_c, 2) );
		double theta_rel = atan2(y - y_c, x - x_c) - atan2(map_parameters_[map_flag_][prev_reg][1] - y_c, map_parameters_[map_flag_][prev_reg][0] - x_c);

		if (theta_rel < -M_PI)
	    	theta_rel += 2.0 * M_PI;
	    else if (theta_rel > M_PI)
	    	theta_rel -= 2.0 * M_PI;

		s_ = map_parameters_[map_flag_][curr_reg][3] + theta_rel / map_parameters_[map_flag_][curr_reg][5];
		n_ = 1.00 / map_parameters_[map_flag_][curr_reg][5] - d_rel;

		d_psi_ = psi - map_parameters_[map_flag_][prev_reg][2] - theta_rel;

		if (d_psi_ < -M_PI){
			while (d_psi_ < -M_PI)
				d_psi_ += 2.0 * M_PI;		
		}
		
		else if (d_psi_ > M_PI){
			while (d_psi_ > M_PI)
				d_psi_ -= 2.0 * M_PI;
		}
	    
	}
	   
	int Map_CA::indentifyRegion(double x, double y)
	{
		// Identify the region by dividing the map into a set of straight lines to denote different regions and finding where the point lies wrt these lines 
		if (map_flag_ == 0)
		{
			if ( y >= (map_parameters_[map_flag_][0][1] - map_parameters_[map_flag_][1][1]) / (map_parameters_[map_flag_][0][0] - map_parameters_[map_flag_][1][0]) * (x - map_parameters_[map_flag_][0][0]) + map_parameters_[map_flag_][0][1])
				return (1);
			else if ( y <= (map_parameters_[map_flag_][3][1] - map_parameters_[map_flag_][2][1]) / (map_parameters_[map_flag_][3][0] - map_parameters_[map_flag_][2][0]) * (x - map_parameters_[map_flag_][2][0]) + map_parameters_[map_flag_][2][1])
				return (3);
			else
			{
				double x01 = 0.5 * (map_parameters_[map_flag_][0][0] + map_parameters_[map_flag_][1][0]);
				double y01 = 0.5 * (map_parameters_[map_flag_][0][1] + map_parameters_[map_flag_][1][1]);
				double x23 = 0.5 * (map_parameters_[map_flag_][2][0] + map_parameters_[map_flag_][3][0]);
				double y23 = 0.5 * (map_parameters_[map_flag_][2][1] + map_parameters_[map_flag_][3][1]);
				double inc = (y01 - y23)/(x01 - x23);

				if (y <= inc * (x-x01) + y01)
	            	return (2);
	        	else if ( y >= - 1.0 /inc * (x - map_parameters_[map_flag_][4][0]) + map_parameters_[map_flag_][4][1] )
	            	return (0);
	        	else
	        		return (4);
	        
			}
		}

		else if (map_flag_ == 1)
		{
			if ( y <= (map_parameters_[map_flag_][0][1] - map_parameters_[map_flag_][1][1]) / (map_parameters_[map_flag_][0][0] - map_parameters_[map_flag_][1][0]) * (x - map_parameters_[map_flag_][0][0]) + map_parameters_[map_flag_][0][1])
				return (1);
			else if ( y >= (map_parameters_[map_flag_][3][1] - map_parameters_[map_flag_][2][1]) / (map_parameters_[map_flag_][3][0] - map_parameters_[map_flag_][2][0]) * (x - map_parameters_[map_flag_][2][0]) + map_parameters_[map_flag_][2][1])
				return (3);
			else
			{
				double x01 = 0.5 * (map_parameters_[map_flag_][0][0] + map_parameters_[map_flag_][1][0]);
				double y01 = 0.5 * (map_parameters_[map_flag_][0][1] + map_parameters_[map_flag_][1][1]);
				double x23 = 0.5 * (map_parameters_[map_flag_][2][0] + map_parameters_[map_flag_][3][0]);
				double y23 = 0.5 * (map_parameters_[map_flag_][2][1] + map_parameters_[map_flag_][3][1]);
				double inc = (y01 - y23)/(x01 - x23);

				if (y >= inc * (x-x01) + y01)
	            	return (2);
	        	else if ( y <= - 1.0 /inc * (x - map_parameters_[map_flag_][4][0]) + map_parameters_[map_flag_][4][1] )
	            	return (0);
	        	else
	        		return (4);
	        
			}	
		}
		
	}

	void Map_CA::wheelSpeedCallback(autorally_msgs::wheelSpeeds speeds)
    {

        lock_.lock();
        wf_ = (speeds.lfSpeed + speeds.rfSpeed) / 2.0;
        wr_ = (speeds.lbSpeed + speeds.rbSpeed) / 2.0;
        lock_.unlock();

    }

};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "MAP_CA");
  autorally_control::Map_CA publish_CA_states;
  ros::spin();
}
