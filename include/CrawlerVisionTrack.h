#ifndef __CRAWLER_VISION_TRACK__
#define __CRAWLER_VISION_TRACK__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

using namespace std;

typedef vector<unsigned char> image_t;

class VisionTracker {

public:
	VisionTracker(ros::NodeHandle &);
	~VisionTracker();
	void Track(size_t rate);

private:
	image_transport::ImageTransport it_;
	image_transport::Subscriber SubImage;
	image_transport::Publisher PubImage;
	// Debug Msgs Publish
	ros::Publisher DebugMsgs;

	void ImageProc(const sensor_msgs::ImageConstPtr& msg);
	
	inline void RGB2V(const sensor_msgs::ImageConstPtr& RGB, image_t& V);

	sensor_msgs::Image curFrame;
	sensor_msgs::Image vChannel;

	bool FIRST_FRAME;

	image_t V;

};

#endif
