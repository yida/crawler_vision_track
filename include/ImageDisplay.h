#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <crawler_vision_track/CrawlerMsgs.h>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageDisplay {
	  image_transport::ImageTransport it_;
	  image_transport::Subscriber image_sub;
	  image_transport::Publisher image_pub;

public:
	 ImageDisplay(ros::NodeHandle& Node);
	 ~ImageDisplay();

	void imageProc(const sensor_msgs::ImageConstPtr& msg);
};
