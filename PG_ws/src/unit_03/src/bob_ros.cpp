// A simple robot named: bob_ros
// Author: University of Bonn, Autonomous Intelligent Systems
//
// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>


#include <std_msgs/Int16.h>

#include <algorithm>
#include <bits/stdc++.h>
#include <math.h>
#include <cmath>
#include <chrono>
#include <queue>
#include <list>
#include "dikstra.cpp"
#include "knoten.cpp"
#include "robo_wall.h"
//#include "Graph.cpp"
#include "Graph.h"
#include <vector>


// Class definition
class bob_ros {
public:
	bob_ros();
	void dest_input();
	void emergencyStop();
	void calculateCommand();
	void mainLoop();
	void avoid_obstical();

	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void amclCallback(const geometry_msgs::PoseWithCovarianceStamped pose);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void handleNavigationNode(std_msgs::Int16);


protected:

	// Nodehandle for bob_ros robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

	// AMCL
	ros::Subscriber m_amclSubscriber;
	geometry_msgs::PoseWithCovarianceStamped amcl_pose;

	geometry_msgs::Pose2D pose_estimate;

	ros::Subscriber navigation_node_subscriber;
	// Queue of absolute positions that the robot should move to in order of the queue
	std::queue<geometry_msgs::Pose2D> navigation_node_queue;
	geometry_msgs::Pose2D *current_navigation_node;


	double older_X = 0;
	double counter = 0;

};

// constructor
bob_ros::bob_ros() {
	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initialising the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &bob_ros::laserCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
	//AMCL
	m_amclSubscriber = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("amcl_pose", 20, &bob_ros::amclCallback,this);
	navigation_node_subscriber = m_nodeHandle.subscribe<std_msgs::Int16>("navigation_node", 20, &bob_ros::handleNavigationNode, this);
}// end of bob_ros constructor

// callback for getting laser values
void bob_ros::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {

	if(	(&m_laserscan)->ranges.size() < scanData->ranges.size()	){
		m_laserscan.ranges.resize(	(size_t)scanData->ranges.size()	);
    	}
	for(unsigned int i = 0; i < scanData->ranges.size(); i++){
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback

//AMCL Callback
void bob_ros::amclCallback(const geometry_msgs::PoseWithCovarianceStamped pose) {

	amcl_pose = pose;
	float	x = amcl_pose.pose.pose.position.x;
	float y = amcl_pose.pose.pose.position.y;
	//ausrichtung = amcl_pose->pose.pose.orientation
	//yaw = tf::getYaw(amcl_pose->pose.pose.orientation);
	//tf2::Matrix3x3(amcl_pose.pose.pose.orientation).getRPY(roll, pitch, yaw);

	geometry_msgs::Quaternion _q = amcl_pose.pose.pose.orientation;
	tf2::Quaternion q(_q.x, _q.y, _q.z, _q.w);
	tf2::Matrix3x3 m(q);
	double roll;
	double pitch;
	double yaw;
	m.getRPY(roll, pitch, yaw);
	float theta = yaw;
	pose_estimate.x = x;
	pose_estimate.y = y;
	pose_estimate.theta = theta;

}


void bob_ros::dest_input()	{
	//knoten auswahlen...
	int start_knoten = 31;
	int ziel_knoten = 38;
	// list of path zuruch von dijkstra bekommen
	list<vertex_t> path = dikstra_main(	start_knoten,	ziel_knoten);

	graph map = answer();
	int path_size = path.size();
	for(int i=0; i<path_size;	i++){
		int element = path.front();
		ROS_INFO("\n	path	%d	\n", element);
		path.pop_front();
		geometry_msgs::Pose2D pose;
    pose.x = map.knotengraph[element][0];
    pose.y = map.knotengraph[element][1];
    pose.theta = 0;
		navigation_node_queue.push(pose);
	}

}
// in terminal: rostopic pub /navigation_node std_msgs/Int16  'ziel'
void bob_ros::handleNavigationNode(std_msgs::Int16 ziel_knoten) {

	int start_knoten = 0;
	int ziel_knotenx = ziel_knoten.data;
	list<vertex_t> path = dikstra_main(	start_knoten,	ziel_knotenx);

	graph map = answer();
	int path_size = path.size();
	for(int i=0; i<path_size;	i++){
		int element = path.front();
		ROS_INFO("\n	path	%d	\n", element);
		path.pop_front();
		geometry_msgs::Pose2D pose;
    pose.x = map.knotengraph[element][0];
    pose.y = map.knotengraph[element][1];
    pose.theta = 0;
		navigation_node_queue.push(pose);
	}

	//ROS_INFO("Queued navigation node { .x = %f, .y = %f, .theta = %f }", navigation_node.x, navigation_node.y, navigation_node.theta);

}
// http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics

void bob_ros::emergencyStop() {
        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0)
	{
		for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
		{
			if( m_laserscan.ranges[i] <= 0.20) {
				m_roombaCommand.linear.x = 0.0;

				return;
			}
		}//end of for loop
	}// end of if
}// end of emergencyStop


void bob_ros::avoid_obstical()	{
	if(!navigation_node_queue.empty())	{
		/*
			float* move;
			move = followwall(m_laserscan, move);
			m_roombaCommand.linear.x = move[0];
			//float one= 1;
			//float two =*(move +1 );
			m_roombaCommand.angular.z = move[1];//	*(move +one );
			return;
			*/
			followwall(m_roombaCommand, m_laserscan);
		}
}

// here we go
// this is the place where we will generate the commands for the robot
void bob_ros::calculateCommand() {

		if (navigation_node_queue.empty())	{
			m_roombaCommand.linear.x = 0.0;
			m_roombaCommand.angular.z = 0.0;
			return;
		}

	geometry_msgs::Pose2D dest = navigation_node_queue.front();

	float factor = 0.10;
	float d_x = fabs(dest.x - pose_estimate.x);
	float d_y = fabs(dest.y - pose_estimate.y);

	if (d_x < factor && d_y < factor) {
		ROS_INFO("\n\nREACHED DESTINATION:	.x = %.3f, .y = %.3f, .theta = %.3f", dest.x, dest.y, dest.theta);
		ROS_INFO("\n\nACTUAL COORDINATES: 	.x = %.3f, .y = %.3f, .theta = %.3f", pose_estimate.x, pose_estimate.y, pose_estimate.theta);
		navigation_node_queue.pop();
		geometry_msgs::Pose2D next_dest = navigation_node_queue.front();
		ROS_INFO("\n\nNEXT DESTINATION:	.x = %.3f, .y = %.3f, .theta = %.3f\n\n", next_dest.x, next_dest.y, next_dest.theta);
	}

	// Do nothing when there is no destination
	if (navigation_node_queue.empty())	{
			m_roombaCommand.linear.x = 0.0;
			m_roombaCommand.angular.z = 0.0;
			return;
		}

	geometry_msgs::Pose2D dest2 = navigation_node_queue.front();

	float diff_x = dest2.x - pose_estimate.x;
	float diff_y = dest2.y - pose_estimate.y;
	float factor_angle = (10.0 / 360) * 2 * M_PI;

	// The angle that directly leads to the destination pose
	float d_theta = atan2f(diff_y, diff_x) - pose_estimate.theta;

	m_roombaCommand.angular.z = 0.2 * tanh(d_theta* 10);
	// Only turn when we are in 45deg range of the destination direction
	m_roombaCommand.linear.x = 0.2 * tanh(10 * (pow(diff_x, 2) + pow(diff_y, 2))) * (fabs(d_theta) < factor_angle);
} // end of calculateCommands


//
void bob_ros::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);



	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok())
	{

		if (counter == 0) {
		//	dest_input();
			counter = 1;
		}

		calculateCommand();
		//avoid_obstical();
		emergencyStop();

		//ROS_INFO(" robot_04 dude runs with: .x=%+6.2f[m/s], .z=%+6.2f[rad/s]", m_roombaCommand.linear.x, m_roombaCommand.angular.z);

		// send the command to the roomrider for execution
		m_commandPublisher.publish(m_roombaCommand);
		if (pose_estimate.x != older_X) {
			ROS_INFO(" Xposition: %.3f - Yposition: %.3f Yaw: %.3f", pose_estimate.x, pose_estimate.y, pose_estimate.theta);
			older_X = pose_estimate.x;
		}
		// spinOnce, just make the loop happen once

		ros::spinOnce();
		// and sleep, to be aligned with the 50ms loop rate
		loop_rate.sleep();
	}
}// end of mainLoop


int main(int argc, char** argv) {
	// initialize
	ros::init(argc, argv, "bob_ros");

	// get an object of type bob_ros and call it dude
	bob_ros dude;

	// main loop
	// make dude execute it's task
	dude.mainLoop();

	return 0;

}// end of main

// end of file: robo_04.cpp
