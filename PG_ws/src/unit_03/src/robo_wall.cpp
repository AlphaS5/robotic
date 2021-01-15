#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include "robo_wall.h"



/*
class xy {
public:
	//Move_and_stop();
	//void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	//void emergencyStop();
	void wallmoves();
	void followwall(int zero);
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


// Subscriber and membervariables for our laserscanner
ros::Subscriber m_laserSubscriber;
sensor_msgs::LaserScan m_laserscan;

// Publisher and membervariables for the driving commands
ros::Publisher m_commandPublisher ;
geometry_msgs::Twist m_roombaCommand;



void wallmoves() {

	if(m_laserscan.ranges.size() == 0) return; // if there is no data

	double c = 0.3; // max speed
	double a = 0.5; //	linear
	double b = 0.3; // rotation
	//double r = m_laserscan.ranges[320];
//	double l = m_laserscan.ranges[220];

	 // tanh for smooth operation
///	double z = b * tanh(r - l);
//	double x = c * tanh(a * (r + l) - 1.2);

	//m_roombaCommand.angular.z = z;
	//m_roombaCommand.linear.x = x;

	for (int i = 220; i < 320; i++) {
		if (m_laserscan.ranges[i] <= 1) {
			m_roombaCommand.angular.z = b;
			m_roombaCommand.linear.x = 0;
		}
	}
} // end of calculateCommands
*/

void followwall(geometry_msgs::Twist m_roombaCommand , 	sensor_msgs::LaserScan m_laserscan) {

	int lazers = 540;
	float maxdis = 0.7; // maximum distances to wall
	float mindis = 0.2; // minimum distances to wall
	int closest_lazer;
	float minvalue = 5;
	float c = 0.3; // speed constsant
	float r = 0.3;
	float degree = 90;
	float dec = 0.3;
	int side = 1; // 1 = go right , -1 = go left

	for (int j = 90; j < 450; j++) {
		//if( m_laserscan.ranges[j] <= 0.20) {		* ( m_laserscan.ranges[j] <= 1)

		if(m_laserscan.ranges.size() == 0) return; // if there is no data
		if( m_laserscan.ranges[j] <= 1.0) {
			m_roombaCommand.linear.x  = 0.3;
			m_roombaCommand.angular.z = 0.3;
		}
			for (int i = 85; i < 95; i++) {
				if(m_laserscan.ranges[i] <= maxdis && m_laserscan.ranges[i] >= mindis && ( m_laserscan.ranges[j] <= dec)){
						m_roombaCommand.linear.x = c; //c is for the speed of light
						m_roombaCommand.angular.z = side *(mindis - m_laserscan.ranges[90]);
						break;
				}
			}

			for (int i = 85; i < 95; i++) {
				if(m_laserscan.ranges[i] >= maxdis && ( m_laserscan.ranges[j] <= dec)){
						//m_roombaCommand.linear.x = c; //c is for the speed of light
						m_roombaCommand.angular.z = -side * r;

						if (m_laserscan.ranges[i - 15] <= maxdis * 2) {
						m_roombaCommand.angular.z = -side * r;//degree * M_PI/180;
						return;
						}
						break;
				}
			}

			for (int i = 180; i < 310; i++){
				if (m_laserscan.ranges[i] <= 0.4 && ( m_laserscan.ranges[j] <= dec)) {
					m_roombaCommand.angular.z = side * r;//degree * M_PI/180;
					m_roombaCommand.linear.x = 0;
				}
			}

			for (int i = 250; i < 280; i++){
				if(m_laserscan.ranges[i] <= maxdis &&   ( m_laserscan.ranges[j] <= dec) && m_laserscan.ranges[90] >= maxdis*2){
				m_roombaCommand.angular.z = side *	r;// degree * M_PI/180;
				break;
				}
			}
	}
}
