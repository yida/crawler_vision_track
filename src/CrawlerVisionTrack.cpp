#include <CrawlerVisionTrack.h>

VisionTracker::VisionTracker(ros::NodeHandle& node)
:it_(node)
,SubImage(it_.subscribe("camera/image_color",1,&VisionTracker::ImageProc, this))
,PubImage(it_.advertise("debug_image",1))
,DebugMsgs(node.advertise<crawler_vision_track::ImageDebug>("debug_msgs",100))

,curFrame()
,debug()
,FIRST_FRAME(true)
,frame_count(0)
{
	GauKer << 0.03222695 << 0.0644530 << 0.12890780 << 0.25781560 << 0.51563120 << 0.25781560 << 0.1289078 << 0.0644539 << 0.03222695 << arma::endr
				 << 0.06445390 << 0.1289070 << 0.25781560 << 0.51563120 << 1.03126250 << 0.51563121 << 0.2578156 << 0.1289078 << 0.06445390 << arma::endr
				 << 0.12890781 << 0.2578150 << 0.51563120 << 1.03126250 << 2.06252500 << 1.03126252 << 0.5156312 << 0.2578156 << 0.12890781 << arma::endr
				 << 0.25781562 << 0.5156310 << 1.03126250 << 2.06252501 << 4.12505000 << 2.06252504 << 1.0312625 << 0.5156312 << 0.25781562 << arma::endr
				 << 0.51563125 << 1.0312620 << 2.06252501 << 4.12505002 << 8.25010000 << 4.12505008 << 2.0625250 << 1.0312625 << 0.51563125 << arma::endr
				 << 0.25781562 << 0.5156310 << 1.03126250 << 2.06252501 << 4.12505002 << 2.06252504 << 1.0312625 << 0.5156312 << 0.25781562 << arma::endr
				 << 0.12890781 << 0.2578150 << 0.51563120 << 1.03126250 << 2.06252501 << 1.03126252 << 0.5156312 << 0.2578156 << 0.12890781 << arma::endr
				 << 0.06445390 << 0.1289070 << 0.25781560 << 0.51563120 << 1.03126250 << 0.51563121 << 0.2578156 << 0.1289078 << 0.06445390 << arma::endr
				 << 0.03222695 << 0.0644539 << 0.12890780 << 0.25781560 << 0.51563120 << 0.25781560 << 0.1289078 << 0.0644539 << 0.03222695 << arma::endr;

}

VisionTracker::~VisionTracker() {
}

//-------------------
// Image Processing
//-------------------
inline bool VisionTracker::RGB2V(const sensor_msgs::ImageConstPtr& RGB, arma::mat& V){
	size_t sizeRGB = RGB->step * RGB->height;
	size_t idxV = 0;
	unsigned char Brightness = 0;
//	ROS_INFO("%d %d",V.n_rows,V.n_cols);
	for (size_t idxRGB = 0; idxRGB < sizeRGB; idxRGB+=3){
		Brightness = RGB->data[idxRGB];
		Brightness = std::max(Brightness,RGB->data[idxRGB+1]);
		Brightness = std::max(Brightness,RGB->data[idxRGB+2]);		
//		ROS_INFO("height %d, width %d",idxV/RGB->width,idxV%RGB->width);
		V(idxV/RGB->width,idxV%RGB->width) = Brightness;
		idxV++;
	} 
	return true;
}

inline bool VisionTracker::GauBlur(const arma::mat& V, arma::mat& Blur) {
//	arma::mat Vs = V.submat(0,0,4,4);		
	double sum;
	size_t kerSize = GauKer.n_rows;
	size_t width = V.n_cols - kerSize;
	size_t height = V.n_rows - kerSize;
	for (size_t r = 0; r < height; r++)
		for (size_t c = 0; c < width; c++) {
			sum = arma::accu(V.submat(r,c,r+kerSize-1,c+kerSize-1) % GauKer);
			Blur(r+kerSize/2,c+kerSize/2) = V(r+kerSize/2,c+kerSize/2) - sum; 
		}
//	Blur.print("acc num:");
//	std::cout << "acc num:" << std::endl << std::cout << Blur << std::endl;
	return 0;
}

//-------------------
// Debug Msg Publish
//-------------------
bool VisionTracker::debugImagePublish(arma::mat& IMG) {
	sensor_msgs::Image debugImg;
	debugImg.header.stamp = ros::Time::now();
	debugImg.header.frame_id = "debug_image";
	debugImg.height = IMG.n_rows;
	debugImg.width = IMG.n_cols;
	debugImg.encoding = "mono8";
	debugImg.is_bigendian = 0;
	debugImg.step = IMG.n_cols;
	arma::mat tIMG = IMG.st();
	double maxIMG = IMG.max();
	double minIMG = IMG.min();
	for (size_t i = 0; i < tIMG.size(); i++) {
		debugImg.data.push_back((char)round((tIMG.at(i)-minIMG)*255/(maxIMG-minIMG)));
	}
	PubImage.publish(debugImg);
	return true;	
}


void VisionTracker::ImageProc(const sensor_msgs::ImageConstPtr& msg){
	std::string imageType = msg->encoding;
	curFrame = *msg;
	arma::mat V_layer(msg->height,msg->width);
	arma::mat V_blur(msg->height,msg->width);

	if (!imageType.compare("bgr8"))
	{
		frame_count ++;
		// Convert RGB to HSV
		if (FIRST_FRAME) {
			ROS_INFO("Image Process Triggered");
			FIRST_FRAME = false;
			// Initiate V layer Size
		}

		RGB2V(msg,V_layer);
		GauBlur(V_layer,V_blur);

		// Debug Msgs
		debug.header.stamp = ros::Time::now();
		debug.header.frame_id = "debug_msgs";
		debug.Vwidth = V_layer.n_cols;
		debug.Vheight = V_layer.n_rows;
		debug.VpackageSize = V_layer.size();
		DebugMsgs.publish(debug);

		// Publish V Channel
		debugImagePublish(V_blur);	

//		GauKer.print("Gau:");
//		std::cout << "Gau:" << std::endl << GauKer << std::endl;
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
