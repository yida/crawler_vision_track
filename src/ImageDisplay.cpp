#include <ImageDisplay.h>


ImageDisplay::ImageDisplay(ros::NodeHandle& node)
:it_(node)
,image_pub(it_.advertise("display_image", 1))
,image_sub(node, "/camera/image_color_local", 1)
,crawler_sub(node, "crawler_msgs", 1)
,sync(image_sub, crawler_sub, 10)
{
	sync.registerCallback(boost::bind(&ImageDisplay::Callback, this, _1, _2));
	cv::namedWindow(WINDOW);
}

ImageDisplay::~ImageDisplay() {
	cv::destroyWindow(WINDOW);
}

void ImageDisplay::Callback(const sensor_msgs::ImageConstPtr& img, 
														const crawler_vision_track::CrawlerMsgs::ConstPtr& crawler) {
	ROS_INFO("Receive Image");
	cv_bridge::CvImagePtr cv_ptr;
  try
  {
     cv_ptr = cv_bridge::toCvCopy(img, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::circle(cv_ptr->image, cv::Point(crawler->centroidX, crawler->centroidY), 10, CV_RGB(255,0,0));

	image_pub.publish(cv_ptr->toImageMsg());
}


int main(int argc, char** argv) {
	  ros::init(argc, argv, "image_display");
		
		ros::NodeHandle nh_;
	  ImageDisplay display(nh_);
	  ros::spin();
	  return 0;
}
