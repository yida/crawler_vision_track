#include <ImageDisplay.h>
#include <CrawlerVisionTrack.h>

std::vector<Crawler> crawler_cache;

ImageDisplay::ImageDisplay(ros::NodeHandle& node)
:it_(node)
,image_sub(it_.subscribe("/camera/image_color_local", 1, &ImageDisplay::ImageCallback, this))
,image_pub(it_.advertise("display_image", 1))
,crawler_sub(node.subscribe("/crawler_msgs", 1, &ImageDisplay::CrawlerCallback, this))
//,image_sub(node, "/camera/image_color_local", 1)
//,crawler_sub(node, "crawler_msgs", 1)
//,sync(image_sub, crawler_sub, 10)
//,alpha(2)
//,beta(100)
{
//	sync.registerCallback(boost::bind(&ImageDisplay::Callback, this, _1, _2));
	cv::namedWindow(WINDOW);
	node.getParam("contrast", alpha);
	ROS_INFO("contrast value %f", alpha);
	node.getParam("brightness", beta);
	ROS_INFO("brightbess value %d", beta);
}

ImageDisplay::~ImageDisplay() {
	cv::destroyWindow(WINDOW);
}

void ImageDisplay::CrawlerCallback(const crawler_vision_track::CrawlerMsgs::ConstPtr& crawler) {
	Crawler curCrawler;
	curCrawler.time = crawler->header.stamp.now().toSec();
	curCrawler.centroidX = crawler->centroidX;
	curCrawler.centroidY = crawler->centroidY;
	crawler_cache.
}

void ImageDisplay::ImageCallback(const sensor_msgs::ImageConstPtr& img) { 
	ROS_INFO("Receive Image and Crawler Info");
	cv_bridge::CvImagePtr cv_ptr;
  try
  {
     cv_ptr = cv_bridge::toCvCopy(img, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

	cv::Mat new_image = cv::Mat::zeros( cv_ptr->image.size(), cv_ptr->image.type() );
	for( int y = 0; y < cv_ptr->image.rows; y++) {
		for( int x = 0; x < cv_ptr->image.cols; x++) {
			for( int c = 0; c < 3; c++) {
				cv_ptr->image.at<cv::Vec3b>(y,x)[c] = 
							cv::saturate_cast<uchar>( alpha*( cv_ptr->image.at<cv::Vec3b>(y,x)[c] ) + beta );
			}
		}
	}

//  cv::circle(cv_ptr->image, cv::Point(crawler->centroidX, crawler->centroidY), 20, CV_RGB(255,0,0));

	image_pub.publish(cv_ptr->toImageMsg());
}


int main(int argc, char** argv) {
	  ros::init(argc, argv, "image_display");
		
		ros::NodeHandle nh_;
	  ImageDisplay display(nh_);
	  ros::spin();
	  return 0;
}
