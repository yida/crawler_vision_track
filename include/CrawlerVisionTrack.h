#ifndef __CRAWLER_VISION_TRACK__
#define __CRAWLER_VISION_TRACK__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

class VisionTracker {

public:
	VisionTracker(ros::NodeHandle &);
	~VisionTracker() {}

private:
	image_transport::ImageTransport it_;
	image_transport::Subscriber SubImage;
	// Debug Msgs Publish
	ros::Publisher DebugMsgs;

	void ImageProc(const sensor_msgs::ImageConstPtr& msg);
	void Tracker();

	sensor_msgs::Image curFrame;

};

#endif
