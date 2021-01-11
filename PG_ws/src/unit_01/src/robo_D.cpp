//
// LAB: Mobile Robotics
//
// unit_01 
// robo_D
//
// A simple robot control named: Mystery_mover 
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Class definition
class Mystery_mover {
public:
	Mystery_mover();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void emergencyStop();
	void calculateCommand();
	void readLaser();
	void mainLoop();

protected:

	// Nodehandle for Mystery_mover robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// constructor
Mystery_mover::Mystery_mover() {

	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initializing the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Mystery_mover::laserCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

}// end of Mystery_mover constructor



// callback for getting laser values 
void Mystery_mover::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
    
	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size()); 
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback



// have a look into the laser measurement 
void Mystery_mover::readLaser() {
	double sum = 0.0 ;

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	{
	   // go through all laser beams
   	   for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
	     {
		sum = sum + m_laserscan.ranges[i] ;
 	     }// end of for all laser beams
	  
	} // end of if we have laser data
}// end of readLaser 


// robot shall stop, in case anything is closer than ... 
void Mystery_mover::emergencyStop() {

	int danger = 0 ;
        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	   for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
	      if( m_laserscan.ranges[i] <= 0.40) 
		 danger = 1 ;		
		
	if(1==danger)
	  {
	    m_roombaCommand.linear.x = 0.0;
 	    m_roombaCommand.angular.z = 0.0;
	    ROS_INFO(" Robot halted by emergency stop" ) ;
	  } // end of if danger
	
}// end of emergencyStop
 


// here we go
// this is the place where we will generate the commands for the robot
void Mystery_mover::calculateCommand() {

	// please find out what this part of the robot controller is doing
	// watch the robot 
	// look into the source file 
	// and try to deduce the underlying idea

        // see if we have laser data available
	if( (&m_laserscan)->ranges.size() > 0)
          {
		double qs = 0.2 ;
                double qt = 0.0 ; 
                int c_r = 0 ;
                int c_l = 0 ;

                // first part
                for( int i=45;i<=270;i++)
                   if (m_laserscan.ranges[i] < 0.5)
                      c_r++; 

                // second part 
                for( int i=270;i<=490;i++)
                   if (m_laserscan.ranges[i] < 0.7)
                      c_l++ ;


                // third part 
                if (c_l > 5)
                   {
                        qs = 0.1, qt  = -0.25 ;
                   } // end of if

                // fourth part 
                if (c_r > 5)
                   {
                        qs = 0.1 ;
                        qt = +0.3 ;
                   } // end of if


                // set the roomba velocities
                // the linear velocity (front direction is x axis) is measured in m/sec
                // the angular velocity (around z axis, yaw) is measured in rad/sec
                m_roombaCommand.linear.x  = qs ;
                m_roombaCommand.angular.z = qt ;

          } // end of if we have laser data



} // end of calculateCommands


//
void Mystery_mover::mainLoop() {
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
		// still not necessary to model anything for this job 

	   /* Plan  */
		// Whatever the task is, here is the place to plan the actions
		calculateCommand();
	
		ROS_INFO(" robo_D commands:forward speed=%f[m/sec] and turn speed =%f[rad/sec]", 
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
	ros::init(argc, argv, "Mystery_mover");

	// get an object of type Mystery_mover and call it robbi
	Mystery_mover robbi;

	// make robbi run
	// main loop
	robbi.mainLoop();

	return 0;
}

// end of file: robo_D.cpp
