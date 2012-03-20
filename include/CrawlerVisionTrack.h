#ifndef __CRAWLER_VISION_TRACK__
#define __CRAWLER_VISION_TRACK__

#include <queue>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <armadillo>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <crawler_vision_track/ImageDebug.h>

struct Crawler {
	double likelihood;
	int centroid_X;
	int centroid_Y;
	int bcentroid_X;
	int bcentroid_Y;
	int gcentroid_X;
	int gcentroid_Y;
};

struct Pixel {
	int width;
	int height;
};

typedef std::vector<Pixel> Mask;

class SortCrawler {
public:
	bool operator() (Crawler& C1, Crawler& C2);
};

class VisionTracker {

public:
	VisionTracker(ros::NodeHandle &);
	~VisionTracker();
	void Track(int rate);

private:
	image_transport::ImageTransport it_;
	image_transport::Subscriber SubImage;
	image_transport::Publisher PubImage;
	image_transport::Publisher PubImagergb;
	image_transport::Publisher PubImagebu;
	image_transport::Publisher PubImagebk;
	image_transport::Publisher PubImagegr;

	// Debug Msgs Publish
	ros::Publisher DebugMsgs;

	void ImageProc(const sensor_msgs::ImageConstPtr& msg);

	bool debugImagePublish(image_transport::Publisher& Pub, arma::mat& IMG, std::string Type);
	inline bool markCrawler(const sensor_msgs::ImageConstPtr& Origin_IMG, const Crawler& cralwer) const;
	inline bool Mask2Gray(const Mask& mask, arma::mat& IMG);
	
	inline bool MaskGenerate(const sensor_msgs::ImageConstPtr& RGB, Mask& BMask, Mask& GMask, Mask& BKMask);
	inline bool RGB2V(const sensor_msgs::ImageConstPtr& RGB, arma::mat& V, const int& centroid_X, const int& centroid_Y, const int& AreaSize);
	inline bool Laplacian(const arma::mat& V, arma::mat& Lap);
	inline bool DetectCrawler(const Mask G, const Mask B, Crawler& crawler);
	sensor_msgs::Image curFrame;

	std::priority_queue<Crawler, std::vector<Crawler>, SortCrawler> Crawlers;

	crawler_vision_track::ImageDebug debug;

	bool FIRST_FRAME;
	int maxCrawlers;
	long frame_count;
	arma::mat GauKer;
	Crawler LastCrawler;
	
	int	BlueMaskThresR;
	int	BlueMaskThresG;
	int	BlueMaskThresB;
	int	GreenMaskThresR;
	int	GreenMaskThresG;
	int	GreenMaskThresB;
	int	BlackMaskThresR;
	int	BlackMaskThresG;
	int	BlackMaskThresB;	

};

#endif
