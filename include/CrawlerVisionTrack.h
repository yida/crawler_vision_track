#ifndef __CRAWLER_VISION_TRACK__
#define __CRAWLER_VISION_TRACK__

#include <string>
#include <cstring>
#include <algorithm>
#include <armadillo>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <crawler_vision_track/ImageDebug.h>

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

	bool debugImagePublish(arma::mat& IMG);
	
	inline bool RGB2V(const sensor_msgs::ImageConstPtr& RGB, arma::mat& V);
	inline bool GauBlur(const arma::mat& V, arma::mat& Blur);
	sensor_msgs::Image curFrame;

	crawler_vision_track::ImageDebug debug;

	bool FIRST_FRAME;

	long frame_count;
	arma::mat GauKer;

};

#endif
