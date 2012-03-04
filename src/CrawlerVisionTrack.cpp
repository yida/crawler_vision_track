#include <string>
#include <cstring>
#include <algorithm>
#include <CrawlerVisionTrack.h>
#include <crawler_vision_track/ImageDebug.h>

VisionTracker::VisionTracker(ros::NodeHandle& node)
:it_(node)
,SubImage(it_.subscribe("camera/image_color",1,&VisionTracker::ImageProc, this))
,PubImage(it_.advertise("image_proc",1))
,DebugMsgs(node.advertise<crawler_vision_track::ImageDebug>("debug_msgs",100))
,curFrame()
,vChannel()

,FIRST_FRAME(true)
,V(NULL)
{
}

VisionTracker::~VisionTracker() {
}

inline void VisionTracker::RGB2V(const sensor_msgs::ImageConstPtr& RGB, image_t& V){
	size_t sizeRGB = RGB->step * RGB->height;
	unsigned char Brightness = 0;
	for (size_t idxRGB = 0; idxRGB < sizeRGB; idxRGB+=3){
		Brightness = RGB->data[idxRGB];
		Brightness = max(Brightness,RGB->data[idxRGB+1]);
		Brightness = max(Brightness,RGB->data[idxRGB+2]);		
		V.push_back(Brightness);
	}
}

void VisionTracker::ImageProc(const sensor_msgs::ImageConstPtr& msg){
	string imageType = msg->encoding;
	curFrame = *msg;
	if (!imageType.compare("bgr8"))
	{
		// Convert RGB to HSV
		if (FIRST_FRAME) {
			ROS_INFO("Image Process Triggered");
			FIRST_FRAME = false;
		}
		RGB2V(msg,&V);
		// Publish V Channel
		vChannel.header.stamp = ros::Time::now();
		vChannel.header.seq = 0;
		vChannel.header.frame_id = "image_debug";
		vChannel.height = msg->height;
		vChannel.width = msg->width;
		vChannel.encoding = "mono8";
		vChannel.is_bigendian = msg->is_bigendian;
		vChannel.step = vChannel.width;
		vChannel.data(V);	
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
