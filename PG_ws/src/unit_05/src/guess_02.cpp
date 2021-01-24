#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_camera/capture.h>

#include <geometry_msgs/Twist.h>

// Klassendefinition
class color_control {
public:
	color_control();
	void mainLoop();
	double R_X, R_Y;
	int C_C;
	double setSpeed( double, double );
	void do_some_magic(sensor_msgs::Image* , int , int , int , int );
	void ImageCallback(const sensor_msgs::Image::ConstPtr &image);
	void InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info);


protected:
//sensor_msgs::image_encodings
	sensor_msgs::Image g_image;
	sensor_msgs::CameraInfo g_camera_info;

	ros::NodeHandle m_nodeHandle;

	// Publisher und Membervariable für die Fahrbefehle
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

} ;

void color_control::ImageCallback(const sensor_msgs::Image::ConstPtr &image)
{
	g_image = *image;
	ROS_INFO("cameraCallback\n");
}

void color_control::InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info)
{
  g_camera_info = *info;
}


color_control::color_control()	{
	//Knoten wird im root-Namespace angelegt

	ros::NodeHandle m_nodeHandle("/");
  ros::Subscriber sub = m_nodeHandle.subscribe<sensor_msgs::Image>("/cv_camera/image_raw", 20,  &color_control::ImageCallback, this);

	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
}


double color_control::setSpeed( double center, double size ){
  double normedCenter = (12.0*center/size) - 6.0;
  double retval = 0.0;

  if ( fabs( normedCenter ) > 1.0 ){
      retval = (trunc( normedCenter )/10.0 );
  }
  return retval;
}

void color_control::do_some_magic( sensor_msgs::Image* img, int red, int green, int blue, int tolerance) {

		if(img->width == 0){
			return;
		}
    int radius = 20 ;

		int channels = 3;// sensor_msgs::image_encodings::numChannels(img->encoding);


    int width = img->width;
		ROS_INFO("width %d\n", width);
    int height = img->height;
			ROS_INFO("height %d\n", height);
		//int channels = encoding->numChannels;
  //  int channels = img->nChannels;
    int step = img->step;

    unsigned char redP = 0, greenP = 0, blueP = 0;
    double S_x = 0.0 ;
    double S_y = 0.0 ;
    int red_difference, green_difference, blue_difference ;

    C_C = 1 ;

    for(int y=0;y<height;y++) {
        for(int x=0;x<width;x++) {
          blueP = img->data[y*step+x*channels+0] ;
          greenP = img->data[y*step+x*channels+1] ;
          redP = img->data[y*step+x*channels+2] ;

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
    //cvCircle( img, center, radius, CV_RGB( 255 - red, 255 - green, 255 - blue ) , 3, 8, 0 );
  //  cvShowImage( "result", img );
}


void color_control::mainLoop() {
	// Bestimmt die Anzahl der Schleifendurchläufe pro Sekunde
	ros::Rate loop_rate(20);

	//CvCapture* capture = 0;
	//const int CAMERA_INDEX = 0;
  //cv::VideoCapture capture(CAMERA_INDEX);
//	cv_camera::Capture capture;//= cv_camera::Capture::capture();
	//bool shot = cv_camera::Capture::capture();

	//	sensor_msgs::Image *frame = &g_image;

	// Kamera 0 öffnen
	//capture = cvCaptureFromCAM( 0 );
	 //sensor_msgs::Image *frame ; //= cv_camera::Capture::getImageMsgPtr();
	//cvNamedWindow( "result", 1 );

	// Schleife bricht ab, wenn der Knoten z.B. ein Kill bekommt
	while (m_nodeHandle.ok())
	{
		     // cv_camera::Capture::publish();


   	 do_some_magic(&g_image , 100, 120, 240, 100);


		 //ROS_INFO( "C_C: %d", C_C );

		 //if (C_C > 50)
		 // {
			m_roombaCommand.linear.x = R_Y;
			m_roombaCommand.angular.z = -R_X;
		 // } else {
		//	m_roombaCommand.linear.x = 0.0;
		//	m_roombaCommand.angular.z = 0.0;
		 // }

	//	ROS_INFO(" Vorwaerts: %f - Drehung: %f", m_roombaCommand.linear.x, m_roombaCommand.angular.z);

		// Schicke die Fahrbefehle an den Roomba
		m_commandPublisher.publish(m_roombaCommand);

	// SpinOnce führt die Schleife einmal aus
	ros::spinOnce();
	// sleep stoppt den Prozess ~50ms, damit die looprate eingehalten wird
	loop_rate.sleep();

	}

//	cvReleaseImage( &frame_copy );
	//cvReleaseCapture( &capture );

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
