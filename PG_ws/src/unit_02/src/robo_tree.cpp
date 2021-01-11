//
// LAB: Mobile Robotics
//
// unit_02
// robo_wallboy
//
// A very simple robot control named: Move_and_stop
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <algorithm>
using namespace std;
#include <math.h>


// Class definition
class Move_and_stop {
public:
	Move_and_stop();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void emergencyStop();
	void calculateCommand();
	void readLaser();
	void mainLoop();

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

// constructor
Move_and_stop::Move_and_stop() {

	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initializing the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Move_and_stop::laserCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

}// end of Move_and_stop constructor



// callback for getting laser values
void Move_and_stop::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {

	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size());
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback



// have a look into the laser measurement
void Move_and_stop::readLaser() {
	double sum = 0.0 ;

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0)
	{
	   // go through all laser beams
   	   for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
	     {
		sum = sum + m_laserscan.ranges[i] ;
 	     }// end of for all laser beams

	   ROS_INFO(" Sum of all measured distances:= %f \n", sum ) ;
	} // end of if we have laser data
}// end of readLaser


// robot shall stop, in case anything is closer than ...
void Move_and_stop::emergencyStop() {

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0)
	{
		for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
		{
			if( m_laserscan.ranges[i] <= 0.60) {
				m_roombaCommand.linear.x = 0.0;
				m_roombaCommand.angular.z = 0.0;
			}// end of if too close
		}// end of for all laser beams
	} // end of if we have laser data
}// end of emergencyStop



// here we go
// this is the place where we will generate the commands for the robot
void Move_and_stop::calculateCommand() {

	if(m_laserscan.ranges.size() == 0) return; // if there is no data

	int x = 15; // number of segments
	double segment[x]; //array initialize
	double maximum; //Largest array value
	double speed = 0.2;

	for (int i = 0; i <= x; i++) { // fill array
		double sum = 0; // sum of lazerlengths per segment
		for (int j = 0; j < 540/x; j++) { //go through number of lazers
			sum += m_laserscan.ranges[j+(i*540/x)];//add length of lazer, shifted
		}
		segment[i]=sum;
	}
	for(int i=1; i<=x; i++){ //Case 1: check if all array elements are equal (no walls)
		if(segment[0] != segment[i]) break; //if one unequal exit
		if(i==x){
			m_roombaCommand.linear.x = speed;//if all equal go straight
			//m_roombaCommand.angular.z = 0.1;//if all equal go straight
			return;
		}//ENTER CODE TO SKIP REST
	}

	maximum = segment[0];
	for (int i = 0; i < x; i++) {
		if (segment[i] >= maximum) maximum = segment[i];
	}
	double max_segments [x];//array for Lazernumber of maximum Segments
	int index=0;//Index for next free space in array
	for(int i=1; i<=x;i++){//go through array of all values
		if(segment[i]==maximum){//if equal to maximum
			max_segments[index]=(i*540/x)+(540/x*0.5);//add Middle Lazer to array
			index++;
		}
	}
	int rotate=max_segments[index/2]; //Pick middle one rounded down
	double degree=(rotate-270)/2.0; //calculate the degree
	double rad=degree*M_PI/180; //in radians
	m_roombaCommand.angular.z = rad; //turn there
	m_roombaCommand.linear.x = speed; //go a bit straight

	//Easy way out: just pick the middle one.



} // end of calculateCommands

/*double max(double segment [int x]){
	double max = segment[0];
	for (int i = 0; i < x; i++) {
		if (segment[i] >= max) {
			max = segment[i];
		}
	}
	return max;
} */
//
void Move_and_stop::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);

	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok())
	{
	   /*  Sense */
		// get all the sensor value that might be useful for controlling the robot
		readLaser();

	   /*  Model */
		// no modeling necessary for this simple job

	   /* Plan  */
	  	// Whatever the task is, here is the place to plan the actions
		calculateCommand();

	  	ROS_INFO(" robo_two movement commands:forward speed=%f[m/sec] and turn speed =%f[rad/sec]",
			m_roombaCommand.linear.x, m_roombaCommand.angular.z);


	   /* Act   */
		// once everything is planned, let's make it happen

		// last chance to stop the robot
		emergencyStop();

		// send the command to the roomrider for execution
		m_commandPublisher.publish(m_roombaCommand);

		// spinOnce, just make the loop happen once
		ros::spinOnce();
		// and sleep, to be aligned with the 50ms loop rate
		loop_rate.sleep();

	}// end of if nodehandle O.K.

}// end of mainLoop


int main(int argc, char** argv) {

	// initialize
	ros::init(argc, argv, "Move_and_stop");

	// get an object of type Move_and_stop and call it robbi
	Move_and_stop robbi;

	// make robbi run
	// main loop
	robbi.mainLoop();

	return 0;
}

// end of file: robo_two.cpp
