#ifndef __CRAWLER_VISION_TRACK__
#define __CRAWLER_VISION_TRACK__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

using namespace std;

class VisionTracker {

public:
	VisionTracker(ros::NodeHandle &);
	~VisionTracker() {}
	void Track(size_t rate);

private:
	image_transport::ImageTransport it_;
	image_transport::Subscriber SubImage;
	image_transport::Publisher PubImage;
	// Debug Msgs Publish
	ros::Publisher DebugMsgs;

	void ImageProc(const sensor_msgs::ImageConstPtr& msg);
	
	inline void RGB2V(const sensor_msgs::ImageConstPtr& RGB, uint8_t *V);

	sensor_msgs::Image curFrame;

};

#endif
