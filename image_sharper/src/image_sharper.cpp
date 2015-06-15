//Code to decrease the brightness and increase the contrast of an 
//image captured by the IR camera on a Kinect 2. 
//Should work on Kinect 1 if the topic names are correct.

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOW[] = "/image_sharper_out";
std::string cam_image_topic;
double alpha;
double beta;
 
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_; 
  image_transport::Publisher image_pub_;
   
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("/image_sharper_out", 1);					  	//declare publisher
    image_sub_ = it_.subscribe(cam_image_topic, 1, &ImageConverter::imageCb, this); 	//declare subscriber to kinect image topic
 
    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
	
    try
    {
      if (enc::isColor(msg->encoding))
	cv_ptr= cv_bridge::toCvCopy(msg, enc::BGR8);   //check whether or not it's a color image
      else
        cv_ptr= cv_bridge::toCvCopy(msg, enc::MONO8); //if not (if using Mono or IR camera, use the MONO encoding)
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
         
    //increase contrast and decrease brightness of image (eg. increase contrast by alpha, change brightness by beta)
    cv_ptr->image.convertTo(cv_ptr->image, -1, alpha, beta);
    
    //show the image in the window    
    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    //publish the image as a ros image to the /image_sharper_out topic  
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};
 
int main(int argc, char *argv[])
{
   ros::init(argc, argv, "image_sharper"); //initialize ROS node
   cam_image_topic=argv[1];
   alpha=atof(argv[2]); //takes contrast constant from launch file (AKA alpha)
   beta=atof(argv[3]); //takes brightness constant from launch file (AKA beta)
   ImageConverter ic; 
   ros::spin();
   return 0;
}
