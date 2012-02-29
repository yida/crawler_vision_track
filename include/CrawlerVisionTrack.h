#ifndef __CRAWLER_VISION_TRACK__
#define __CRAWLER_VISION_TRACK__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

class VisionTracker {

public:
	VisionTracker(ros::NodeHandle &);
	~VisionTracker() {}
	void Track(size_t rate);

private:
	image_transport::ImageTransport it_;
	image_transport::Subscriber SubImage;
	// Debug Msgs Publish
	ros::Publisher DebugMsgs;

	void ImageProc(const sensor_msgs::ImageConstPtr& msg);

	cv_bridge::CvImage curFrame;

};

#endif
