#include <CrawlerVisionTrack.h>

bool SortCrawler::operator() (Crawler& C1, Crawler& C2) {
	if (C1.likelihood < C2.likelihood)
		return true;
	else
		return false;
}

VisionTracker::VisionTracker(ros::NodeHandle& node)
:it_(node)
,SubImage(it_.subscribe("camera/image_color",1,&VisionTracker::ImageProc, this))
,PubImage(it_.advertise("debug_image",1))
,PubImagebu(it_.advertise("debug_image_bumask",1))
,PubImagebk(it_.advertise("debug_image_bkmask",1))
,PubImagegr(it_.advertise("debug_image_grmask",1))
,DebugMsgs(node.advertise<crawler_vision_track::ImageDebug>("debug_msgs",100))

,curFrame()
,debug()
,FIRST_FRAME(true)
,maxCrawlers(10)
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
	GauKer = GauKer/arma::accu(GauKer);

	node.getParam("BlueMaskThreshold/Red",BlueMaskThresR);
	node.getParam("BlueMaskThreshold/Green",BlueMaskThresG);
	node.getParam("BlueMaskThreshold/Blue",BlueMaskThresB);
	node.getParam("GreenMaskThreshold/Red",GreenMaskThresR);
	node.getParam("GreenMaskThreshold/Green",GreenMaskThresG);
	node.getParam("GreenMaskThreshold/Blue",GreenMaskThresB);
	node.getParam("BlackMaskThreshold/Red",BlackMaskThresR);
	node.getParam("BlackMaskThreshold/Green",BlackMaskThresG);
	node.getParam("BlackMaskThreshold/Blue",BlackMaskThresB);
}

VisionTracker::~VisionTracker() {
}

//-------------------
// Image Processing
//-------------------
inline bool VisionTracker::MaskGenerate(const sensor_msgs::ImageConstPtr& RGB, Mask& BMask, Mask& GMask, Mask& BKMask) {
	size_t sizeRGB = RGB->step * RGB->height;
	size_t idxV = 0;
	Pixel px;
	for (size_t idxRGB = 0; idxRGB < sizeRGB; idxRGB+=3){
		if ((RGB->data[idxRGB]>=BlueMaskThresB) && 
				(RGB->data[idxRGB+1]<BlueMaskThresG) && 
				(RGB->data[idxRGB+2]<BlueMaskThresR)) {
			px.height = idxV/RGB->width;
			px.width = idxV%RGB->width;
			BMask.push_back(px);
		}

		if ((RGB->data[idxRGB]<GreenMaskThresB) &&
				(RGB->data[idxRGB+1]>=GreenMaskThresG) &&
				(RGB->data[idxRGB+2]<GreenMaskThresR)) {
			px.height = idxV/RGB->width;
			px.width = idxV%RGB->width;
			GMask.push_back(px);
		}

		if ((RGB->data[idxRGB]<BlackMaskThresB) && 
				(RGB->data[idxRGB+1]<BlackMaskThresG) && 
				(RGB->data[idxRGB+2]<BlackMaskThresR)) { 
			px.height = idxV/RGB->width;
			px.width = idxV%RGB->width;
			BKMask.push_back(px);
		}
		idxV++;
	} 
//	std::cout << "blueMask:" << BMask.size() << " GreenMask:" << GMask.size() << std::endl; 
	return true;
}


inline bool VisionTracker::RGB2V(const sensor_msgs::ImageConstPtr& RGB, arma::mat& V){
	size_t sizeRGB = RGB->step * RGB->height;
	size_t idxV = 0;
	unsigned char Brightness = 0;
	for (size_t idxRGB = 0; idxRGB < sizeRGB; idxRGB+=3){
		Brightness = RGB->data[idxRGB];
		Brightness = std::max(Brightness,RGB->data[idxRGB+1]);
		Brightness = std::max(Brightness,RGB->data[idxRGB+2]);		
		V(idxV/RGB->width,idxV%RGB->width) = Brightness;
		idxV++;
	} 
	return true;
}

inline bool VisionTracker::Laplacian(const arma::mat& V, arma::mat& Lap) {
	double sum;
	size_t kerSize = GauKer.n_rows;
	size_t width = V.n_cols - kerSize;
	size_t height = V.n_rows - kerSize;
	for (size_t r = 0; r < height; r++)
		for (size_t c = 0; c < width; c++) {
			sum = arma::accu(V.submat(r,c,r+kerSize-1,c+kerSize-1) % GauKer);
			Lap(r+kerSize/2,c+kerSize/2) = abs(sum - V(r+kerSize/2,c+kerSize/2));
		}
	return 0;
}

inline bool VisionTracker::DetectCrawler(const Mask G, const Mask B, Crawler& crawler) {
	size_t sumWidth = 0;
	size_t sumHeight = 0;
	if (G.size() > 0) {
		for (size_t iter = 0; iter < G.size(); iter++) {
			sumWidth += G[iter].width;
			sumHeight += G[iter].height;
		}
		std::cout << "Found Green LED at height:" << sumHeight/G.size() << " width:" << sumWidth/G.size() << std::endl;
	}
	sumWidth = 0;
	sumHeight = 0;
	if (B.size() > 0) {
		for (size_t iter = 0; iter < B.size(); iter++) {
			sumWidth += B[iter].width;
			sumHeight += B[iter].height;
		}
		std::cout << "Found Blue LED at height:" << sumHeight/B.size() << " width:" << sumWidth/B.size() << std::endl;
	}

	return true;
}

//-------------------
// Debug Msg Publish
//-------------------
inline bool VisionTracker::Mask2Gray(const Mask& mask, arma::mat& IMG) {
	IMG.fill(0.0);
	for (size_t iter = 0; iter < mask.size(); iter++) {
		IMG(mask[iter].height,mask[iter].width) = 1;
	}
	return true;	
}

inline bool VisionTracker::markCrawler(const std::vector<unsigned char>& IMG, const Crawler& crawler, arma::mat& IMG_Label) {
//	IMG_Label = IMG;
//	int cenX = 100;
//	int cenY = 100;
//	int winSize = 20;
//	int width,height;
//	int wIMG = IMG_Label.n_cols;
//	int hIMG = IMG_Label.n_rows;
//	// Draw Square Marker
//	// top line
//	height = std::max(0, cenY - winSize);
//	for (int w = std::max(0, cenX - winSize); w < std::min(wIMG,cenX + winSize); w++) {
//		IMG_Label(height,w) = IMG_Label.min();
//		IMG_Label(height-1,w) = IMG_Label.min();
//		IMG_Label(height+1,w) = IMG_Label.min();
//	}
//	// bottom line
//	height = std::min(hIMG, cenY + winSize);
//	for (int w = std::max(0, cenX - winSize); w < std::min(wIMG,cenX + winSize); w++) {
//		IMG_Label(height,w) = IMG_Label.min();
//		IMG_Label(height-1,w) = IMG_Label.min();
//		IMG_Label(height+1,w) = IMG_Label.min();
//	}
//	// left line
//	width = std::max(0, cenX - winSize);
//	for (int h = std::max(0, cenY - winSize); h < std::min(hIMG,cenY + winSize); h++) {
//		IMG_Label(h,width) = IMG_Label.min();
//		IMG_Label(h,width-1) = IMG_Label.min();
//		IMG_Label(h,width+1) = IMG_Label.min();
//	}
//	// right line
//	width = std::min(hIMG, cenX + winSize);
//	for (int h = std::max(0, cenY - winSize); h < std::min(hIMG,cenY + winSize); h++) {
//		IMG_Label(h,width) = IMG_Label.min();
//		IMG_Label(h,width+1) = IMG_Label.min();
//		IMG_Label(h,width-1) = IMG_Label.max();
//	}
	return true;
}

bool VisionTracker::debugImagePublish(image_transport::Publisher& Pub, arma::mat& IMG, std::string Type) {
	sensor_msgs::Image debugImg;
	debugImg.header.stamp = ros::Time::now();
	debugImg.header.frame_id = "debug_image";
	debugImg.height = IMG.n_rows;
	debugImg.width = IMG.n_cols;
	debugImg.encoding = Type;
	debugImg.is_bigendian = 0;
	debugImg.step = IMG.n_cols;
	arma::mat tIMG = IMG.st();
	double maxIMG = IMG.max();
	double minIMG = IMG.min();
	for (size_t i = 0; i < tIMG.size(); i++) {
		debugImg.data.push_back((char)round((tIMG.at(i)-minIMG)*255/(maxIMG-minIMG)));
	}
	Pub.publish(debugImg);
	return true;	
}


void VisionTracker::ImageProc(const sensor_msgs::ImageConstPtr& msg){
	std::string imageType = msg->encoding;
	curFrame = *msg;
	Crawler crawler;
	arma::mat V_layer(msg->height,msg->width);
	arma::mat V_lap(msg->height,msg->width);
	Mask Blue_Mask;
	Mask Green_Mask;
	Mask Black_Mask;

	if (!imageType.compare("bgr8"))
	{
		frame_count ++;
		// Convert RGB to HSV
		if (FIRST_FRAME) {
			ROS_INFO("Image Process Triggered");
			FIRST_FRAME = false;
			// Initiate V layer Size
		}

		MaskGenerate(msg,Blue_Mask,Green_Mask,Black_Mask);
		RGB2V(msg,V_layer);
		Laplacian(V_layer,V_lap);
		DetectCrawler(Green_Mask,Blue_Mask,crawler);


		// Debug Msgs
		debug.header.stamp = ros::Time::now();
		debug.header.frame_id = "debug_msgs";
		debug.Vwidth = V_layer.n_cols;
		debug.Vheight = V_layer.n_rows;
		debug.VpackageSize = V_layer.size();
		DebugMsgs.publish(debug);
		
			// Publish Debug Image
//		debugImagePublish(PubImage,V_lap,"mono8");
		arma::mat BMask_Gray(msg->height,msg->width);
		Mask2Gray(Blue_Mask,BMask_Gray);
		debugImagePublish(PubImagebu,BMask_Gray,"mono8");
		arma::mat GMask_Gray(msg->height,msg->width);
		Mask2Gray(Green_Mask,GMask_Gray);
		debugImagePublish(PubImagegr,GMask_Gray,"mono8");	
		arma::mat BKMask_Gray(msg->height,msg->width);
		Mask2Gray(Black_Mask,BKMask_Gray);
		debugImagePublish(PubImagebk,BKMask_Gray,"mono8");
		
		arma::mat IMG_Label(msg->height,msg->step);
		markCrawler(msg->data,crawler,IMG_Label);
		debugImagePublish(PubImage,IMG_Label,"bgr8");
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
