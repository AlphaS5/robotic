//
// LAB: Mobile Robotics
//
// unit_01 
// robo_C
//
// A very simple robot control named: Random_mover 
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Class definition
class Random_mover {
public:
	Random_mover();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void emergencyStop();
	void calculateCommand();
	void readLaser();
	void mainLoop();

protected:

	// Nodehandle for Random_mover robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// constructor
Random_mover::Random_mover() {

	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initializing the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Random_mover::laserCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

	// Initialize the random generator 
	// take the time to get random results 
	   // srand(unsigned(time(0)));
	// or take a   seed   to be able to reproduce the results
	srand(21);    // seed

}// end of Random_mover constructor



// callback for getting laser values 
void Random_mover::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
    
	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size()); 
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback



// have a look into the laser measurement 
void Random_mover::readLaser() {
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
void Random_mover::emergencyStop() {

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
void Random_mover::calculateCommand() {

	static int step_count = 0 ;
	double r ;

	// depending on the step_counter, do different things
	if (step_count <= 0) 
	  {
	     // step_count is 0 , time to do something new
	     // step_count is negative should never happen, but just in case :-)

	     // depending on a random value, we do different things
	     r = (double) rand() / RAND_MAX ;
	     ROS_INFO ("r=%lf", r) ;

	     // turn for a while, move forward for a while, continue the last command for a while, ...
	        // default case, if r<= (first value) 
		// don't do anything new
		// no change of roombaCommands
		step_count = 50 ;
	     if ( r > 0.1 ) // move ahead
		{
	 	  m_roombaCommand.linear.x = 0.1;
                  m_roombaCommand.angular.z = 0.0;
		  step_count = 50 ;
		} 
	     if ( r > 0.3 ) //move ahead for a random no of steps 
		{
	 	  m_roombaCommand.linear.x = 0.2;
                  m_roombaCommand.angular.z = 0.0;
		  step_count = 10 + 50.0*rand()/RAND_MAX;
		} 
	     if ( r > 0.5 ) // turn left for a random no of steps 
		{
	 	  m_roombaCommand.linear.x = 0.0;
                  m_roombaCommand.angular.z = 0.15;
		  step_count = 20 + 20.0*rand()/RAND_MAX;
		} 
	     if ( r > 0.7 ) // turn right for a random no of steps 
		{
	 	  m_roombaCommand.linear.x = 0.0;
                  m_roombaCommand.angular.z = -0.1;
		  step_count = 10 + 30.0*rand()/RAND_MAX;
		} 
	  } // end of if (step_count<=0)

	else // so, step_count is positive, which imples that a movement is going on
          { 
	    // just decrease the counter and continue doing whatever has been set before	
	    step_count = step_count -1 ;   
	  } // end of else
	  
	  
} // end of calculateCommands


//
void Random_mover::mainLoop() {
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

	  	ROS_INFO(" robo_C speed: forward=%f and turn=%f", 
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
	ros::init(argc, argv, "Random_mover");

	// get an object of type Random_mover and call it robbi
	Random_mover robbi;

	// make robbi run
	// main loop
	robbi.mainLoop();

	return 0;
}

// end of file: robo_C.cpp
