#include <string>
#include <algorithm>
#include <CrawlerVisionTrack.h>

VisionTracker::VisionTracker(ros::NodeHandle& node)
:it_(node)
,SubImage(it_.subscribe("/camera/image_color",1,&VisionTracker::ImageProc, this))
,DebugMsgs(node.advertise<std_msgs::String>("debug",100))
,curFrame()
{
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
//	ROS_INFO("%s",imageType.c_str());
	if (!imageType.compare("bgr8"))
	{
//		ROS_INFO("Width: %d,Height: %d",msg->width,msg->height);
//		ROS_INFO("Full Row Length in Byte %d",msg->step);
//		ROS_INFO("%s",imageType.c_str());
		// Convert RGB to HSV
		uint8_t *V = new uint8_t [msg->width*msg->height];
		double begin, end;
		begin = ros::Time::now().toSec();
		RGB2V(msg,V);
		end = ros::Time::now().toSec() - begin;
		std::cout << end << std::endl; 
		
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
