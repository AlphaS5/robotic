#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <array>

using namespace std;



	//void wallmoves();
	void followwall(geometry_msgs::Twist m_roombaCommand,
			sensor_msgs::LaserScan m_laserscan);
	//float* followwall(sensor_msgs::LaserScan m_laserscan, float* move);
