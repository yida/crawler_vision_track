#include <ImageDisplay.h>

ImageDisplay::ImageDisplay(ros::NodeHandle& node)
:it_(node)
,image_sub(it_.subscribe("image_color", 1, &ImageDisplay::imageProc, this))
,image_pub(it_.advertise("display_image", 1))
{
	cv::namedWindow(WINDOW);
}

ImageDisplay::~ImageDisplay() {
	cv::destroyWindow(WINDOW);
}

void ImageDisplay::imageProc(const sensor_msgs::ImageConstPtr& msg) {
	ROS_INFO("Receive Image");
	cv_bridge::CvImagePtr cv_ptr;
  try
  {
     cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

	image_pub.publish(cv_ptr->toImageMsg());
}


int main(int argc, char** argv) {
	  ros::init(argc, argv, "image_display");
		
		ros::NodeHandle nh_;
	  ImageDisplay display(nh_);
	  ros::spin();
	  return 0;
}
