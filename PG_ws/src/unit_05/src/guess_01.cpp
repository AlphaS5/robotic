//
// Projektgruppe Multi Robotik
//
// guess what i am doing
// Programm zur unit_05
//

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <math.h>

#include <geometry_msgs/Twist.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


// Klassendefinition
class color_control {
public:
	void mainLoop();
	double R_X, R_Y;
	int C_C;
	double setSpeed( double, double );
	void do_some_magic( IplImage* , int , int , int , int );

	color_control()
	{
	  //Knoten wird im root-Namespace angelegt
	  ros::NodeHandle m_nodeHandle("/");

	  m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
	}

	~color_control()
	{
	  cvDestroyWindow("result");
	}

protected:

	ros::NodeHandle m_nodeHandle;

	// Publisher und Membervariable für die Fahrbefehle
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

} ;


double color_control::setSpeed( double center, double size ){
  double normedCenter = (12.0*center/size) - 6.0;
  double retval = 0.0;

  if ( fabs( normedCenter ) > 1.0 ){
      retval = (trunc( normedCenter )/10.0 );
  }
  return retval;

}

// call me like: do_some_magic( frame , 225, 50, 75, 100);
// comments R 4 mollycoddles, guess what i'm doing!
void color_control::do_some_magic( IplImage* img, int red, int green, int blue, int tolerance) {

    int radius = 20 ;

    int width = img->width;
    int height = img->height;
    int channels = img->nChannels;
    int step = img->widthStep;

    unsigned char redP = 250, greenP = 0, blueP = 0;
    double S_x = 0.0 ;
    double S_y = 0.0 ;
    int red_difference, green_difference, blue_difference ;

    C_C = 0 ;

    for(int y=0;y<height;y++) {
        for(int x=0;x<width;x++) {
          blueP = img->imageData[y*step+x*channels+0] ;
          greenP = img->imageData[y*step+x*channels+1] ;
          redP = img->imageData[y*step+x*channels+2] ;

          {
            red_difference = fabs(red - (int)redP);
            green_difference = fabs(green - (int)greenP);
            blue_difference = fabs(blue - (int)blueP);

            if ( (red_difference + green_difference + blue_difference) < tolerance )
              {
                C_C++ ;
                S_x += x;
                S_y += y;
              }
          }
        }
     }

    S_x = S_x / (C_C) ;
    S_y = S_y / (C_C) ;

    R_X = setSpeed( S_x, img->width );
    R_Y = setSpeed( S_y, img->height );

    CvPoint center;
    center.x = S_x;
    center.y = S_y;
    cvCircle( img, center, radius, CV_RGB( 255 - red, 255 - green, 255 - blue ) , 3, 8, 0 );
    cvShowImage( "result", img );
}

// Hauptschleife
void color_control::mainLoop() {
	// Bestimmt die Anzahl der Schleifendurchläufe pro Sekunde
	ros::Rate loop_rate(20);

	CvCapture* capture = 0;
	IplImage *frame, *frame_copy = 0;

	// Kamera 0 öffnen
	capture = cvCaptureFromCAM( 0 );

	cvNamedWindow( "result", 1 );

	// Schleife bricht ab, wenn der Knoten z.B. ein Kill bekommt
	while (m_nodeHandle.ok())
	{
		    if( !cvGrabFrame( capture )){
		        break;
		    }

		    frame = cvQueryFrame( capture );
		    if( !frame ) {
		        break;
		    }

		    if( !frame_copy )
		        frame_copy = cvCreateImage( cvSize(frame->width,frame->height),
		                                    IPL_DEPTH_8U, frame->nChannels );
		    if( frame->origin == IPL_ORIGIN_TL ){
		        cvCopy( frame, frame_copy, 0 );
		    }
		    else {
		        cvFlip( frame, frame_copy, 0 );
		    }

		    cvShowImage( "result", frame_copy );

            do_some_magic( frame , 100, 120, 240, 100);

		    if( cvWaitKey( 10 ) >= 0 )
		        break;

		 ROS_INFO( "C_C: %d", C_C );

		 if (C_C > 50)
		  {
			m_roombaCommand.linear.x = R_Y;
			m_roombaCommand.angular.z = -R_X;
		  } else {
			m_roombaCommand.linear.x = 0.0;
			m_roombaCommand.angular.z = 0.0;
		  }

		ROS_INFO(" Vorwaerts: %f - Drehung: %f", m_roombaCommand.linear.x, m_roombaCommand.angular.z);

		// Schicke die Fahrbefehle an den Roomba
		m_commandPublisher.publish(m_roombaCommand);

	// SpinOnce führt die Schleife einmal aus
	ros::spinOnce();
	// sleep stoppt den Prozess ~50ms, damit die looprate eingehalten wird
	loop_rate.sleep();

	}

	cvReleaseImage( &frame_copy );
	cvReleaseCapture( &capture );

}

int main(int argc, char** argv) {

	// Initialize
	ros::init(argc, argv, "color_control");
	ros::NodeHandle n;
	color_control f_g;
	f_g.mainLoop();
	ros::spin();
	return 0;

}
