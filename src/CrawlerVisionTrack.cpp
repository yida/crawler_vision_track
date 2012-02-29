#include <string>
#include <CrawlerVisionTrack.h>

VisionTracker::VisionTracker(ros::NodeHandle& node)
:it_(node)
,SubImage(it_.subscribe("/camera/image_color",1,&VisionTracker::ImageProc, this))
,DebugMsgs(node.advertise<std_msgs::String>("debug",100))
,curFrame()
{
}

void VisionTracker::ImageProc(const sensor_msgs::ImageConstPtr& msg)
{
	string imageType = msg->encoding;
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, imageType);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat cv_hsv;
//	ROS_INFO("%s",imageType.c_str());
	if (!imageType.compare("bgr8"))
	{
//		ROS_INFO("Width: %d,Height: %d",cv_ptr->image.rows,cv_ptr->image.cols);
//		ROS_INFO("Full Row Length in Byte %d",cv_ptr->image.step);
//	ROS_INFO("%s",imageType.c_str());
		// Convert RGB to HSV
		cvtColor(cv_ptr->image,cv_hsv->image,CV_BGR2HSV);
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
