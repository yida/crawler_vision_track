
#include <CrawlerVisionTrack.h>

VisionTracker::VisionTracker(ros::NodeHandle& node)
:it_(node)
,SubImage(it_.subscribe("/image_in",1,&VisionTracker::ImageProc, this))
,DebugMsgs(node.advertise<std_msgs::String>("debug",100))

,curFrame()
{
}

void VisionTracker::ImageProc(const sensor_msgs::ImageConstPtr& msg)
{
	curFrame = *msg;
	std::cout << curFrame.encoding << std::endl;
}

void VisionTracker::Tracker()
{
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "crawler_vision_track");
	ros::NodeHandle nh_("/crawler_vision_track");
	VisionTracker tracker(nh_);
	return 0;
}
