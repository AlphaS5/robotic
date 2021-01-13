#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include "robo_wall.h"

using namespace std;
/*
class Move_and_stop {
public:
//	Move_and_stop();
//	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
//	void emergencyStop();
	void calculateCommand();
//	void readLaser();
//	void mainLoop();

protected:

	// Nodehandle for Move_and_stop robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};
*/
// Subscriber and membervariables for our laserscanner
ros::Subscriber m_laserSubscriber;
sensor_msgs::LaserScan m_laserscan;

// Publisher and membervariables for the driving commands
ros::Publisher m_commandPublisher;
geometry_msgs::Twist m_roombaCommand;



void wallmoves() {
	if(m_laserscan.ranges.size() == 0) return; // if there is no data

	double c = 0.3; // max speed
	double a = 0.5; //	linear
	double b = 0.3; // rotation
	double r = m_laserscan.ranges[255];
	double l = m_laserscan.ranges[285];

	 // tanh for smooth operation
	double z = b * tanh(r - l);
	double x = c * tanh(a * (r + l) - 1.2);

	m_roombaCommand.angular.z = z;
	m_roombaCommand.linear.x = x;

  // now this is to make it get closer to a wall if its on it's sides
	if(m_laserscan.ranges[90] <= 5 && m_laserscan.ranges[270] >= 4){ // left side
			m_roombaCommand.angular.z = -0.5 * 3.141592;
	}
	if(m_laserscan.ranges[450] <= 5 && m_laserscan.ranges[270] >= 4){ // right side
			m_roombaCommand.angular.z = 0.5 * 3.141592;
	}
	// this should make it get out of a corner
	if(m_laserscan.ranges[270] <= 1.2 && r == l) {
	 		m_roombaCommand.angular.z = -1*z;
 	}
} // end of calculateCommands
