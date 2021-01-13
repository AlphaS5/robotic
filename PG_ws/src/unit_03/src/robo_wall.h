#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <algorithm>
#include <math.h>

using namespace std;

class Move_and_stop {
public:
	//Move_and_stop();
	//void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	//void emergencyStop();

	//void readLaser();
	//void mainLoop();

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


	void wallmoves();
