#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <crawler_vision_track/CrawlerMsgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageDisplay {
	  image_transport::ImageTransport it_;
	  image_transport::Subscriber image_sub;
	  image_transport::Publisher image_pub;
		ros::Subscriber crawler_sub;
//		message_filters::Subscriber<sensor_msgs::Image> image_sub;
//		message_filters::Subscriber<crawler_vision_track::CrawlerMsgs> crawler_sub;
//		message_filters::TimeSynchronizer<sensor_msgs::Image, crawler_vision_track::CrawlerMsgs> sync;

public:
	 ImageDisplay(ros::NodeHandle& Node);
	 ~ImageDisplay();

//	void Callback(const sensor_msgs::ImageConstPtr& img, 
//								const crawler_vision_track::CrawlerMsgs::ConstPtr& crawler);
	void ImageCallback(const sensor_msgs::ImageConstPtr& img);
	void CrawlerCallback(const crawler_vision_track::CrawlerMsgs::ConstPtr& crawler);
	double alpha;
	int beta;
};
