#include <string>
#include <cstring>
#include <algorithm>
#include <CrawlerVisionTrack.h>
#include <crawler_vision_track/ImageDebug.h>

VisionTracker::VisionTracker(ros::NodeHandle& node)
:it_(node)
,SubImage(it_.subscribe("image_in",1,&VisionTracker::ImageProc, this))
,PubImage(it_.advertise("image_proc",1))
,DebugMsgs(node.advertise<crawler_vision_track::ImageDebug>("debug_msgs",100))
,curFrame()
,vChannel()

,FIRST_FRAME(true)
,V(NULL)
{
}

VisionTracker::~VisionTracker() {
	if (!FIRST_FRAME) {
		delete [] V;
	}
}

inline void VisionTracker::RGB2V(const sensor_msgs::ImageConstPtr& RGB,uint8_t *V){
	size_t idxV = 0;
	size_t sizeRGB = RGB->step * RGB->height;
	for (size_t idxRGB = 0; idxRGB < sizeRGB; idxRGB+=3){
		V[idxV] = RGB->data[idxRGB];
		V[idxV] = max(V[idxV],RGB->data[idxRGB+1]);
		V[idxV] = max(V[idxV],RGB->data[idxRGB+2]);		
	}
}

void VisionTracker::ImageProc(const sensor_msgs::ImageConstPtr& msg)
{
	string imageType = msg->encoding;
	curFrame = *msg;
	if (!imageType.compare("bgr8"))
	{
		// Convert RGB to HSV
		if (FIRST_FRAME) {
			V = new uint8_t [msg->width*msg->height];
			FIRST_FRAME = false;
		}
		RGB2V(msg,V);

		// Publish V Channel
		vChannel.header.stamp = ros::Time::now();
		vChannel.header.seq = 0;
		vChannel.header.frame_id = "image_debug";
		vChannel.height = msg->height;
		vChannel.width = msg->width;
		vChannel.encoding = "mono8";
		vChannel.is_bigendian = msg->is_bigendian;
		vChannel.step = vChannel.width;
		memcpy((void *)&vChannel.data,(void *)V,sizeof(uint8_t)*vChannel.width*vChannel.height);
		PubImage.publish(vChannel);
	}
}

void VisionTracker::Track(size_t rate)
{
	ros::Rate loop_rate(rate);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "crawler_vision_track");
	ros::NodeHandle nh_;
	VisionTracker tracker(nh_);

	ROS_INFO("Setup Done!");

	tracker.Track(15);
	return 0;
}
