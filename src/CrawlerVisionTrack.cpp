#include <string>
#include <cstring>
#include <algorithm>
#include <CrawlerVisionTrack.h>
#include <crawler_vision_track/ImageDebug.h>

ImageProc::ImageProc()
:width(0)
,height(0)
,data()
{}

ImageProc::ImageProc(size_t w,size_t h)
:width(w)
,height(h)
,data()
{
	data.resize(width * height, 0);
}

VisionTracker::VisionTracker(ros::NodeHandle& node)
:it_(node)
,SubImage(it_.subscribe("camera/image_color",1,&VisionTracker::ImageProc, this))
,PubImage(it_.advertise("image_proc",1))
,DebugMsgs(node.advertise<crawler_vision_track::ImageDebug>("debug_msgs",100))
,curFrame()
,vChannel()
,FIRST_FRAME(true)
,V_layer()
,V_laplacian()
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
		V.data[idxRGB/3] = Brightness;
	}
}

inline void VisionTracker::Laplacian(const image_t& IMG, image_t& LAP) {
	size_t lapCoef = 5; // 2 5 10 
	size_t sizeIMG = IMG.width * IMG.height;
	size_t baseWidth, baseHeight, baseIdx;
	size_t width, height;
	for (size_t idxIMG = 0; idxIMG < sizeIMG; idxIMG++) {
		height = idxIMG / lapCoef;
		width = idxIMG % lapCoef;
		baseHeight = height / lapCoef;
		baseWidth = width / lapCoef;
		baseIdx =  * (baseHeight + lapCoef/2) + (baseWidth + lapCoef/2);
		LAP[idxIMG] = abs(IMG[idxIMG]-IMG[baseIdx]);
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
			// Initiate V layer Size
		}
		ImageProc V_layer(msg->width,msg->height);
		ImageProc V_laplacian(msg->width,msg->height);

		RGB2V(msg,V_layer);
		Laplacian(V_layer,V_laplacian);
		// Debug Msgs
		crawler_vision_track::ImageDebug debug;
		debug.Vwidth = V_layer.width;
		debug.Vheight = V_layer.height;
		debug.VpackageSize = V_layer->size();
		DebugMsgs.publish(debug);

		// Publish V Channel
		vChannel.header.stamp = ros::Time::now();
		vChannel.header.seq = 0;
		vChannel.header.frame_id = "image_debug";
		vChannel.height = V_layer->height;
		vChannel.width = V_layer->width;
		vChannel.encoding = "mono8";
		vChannel.is_bigendian = msg->is_bigendian;
		vChannel.step = vChannel.width;
		vChannel.data = V_laplaciani.data;	
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
