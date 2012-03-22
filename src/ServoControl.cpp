#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <string>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

char command9[30];
char command10[30];

void servo_exit(void) {
	system("/sbin/rmmod /home/root/pwm.ko");
}

void servoCallback(const std_msgs::String::ConstPtr& msg) {
	std::cout << msg->data << std::endl;
//	system(msg->data.c_str());

}

//void servoCallbacl(const geometry_msgs::Point:ConstPtr& msg) {
//	sprintf(command9, "echo %d > /dev/pwm9", (int)msg->x);
//	system(command9);
//
//	sprintf(command10, "echo %d > /dev/pwm10", (int)msg->y);
//	system(command10);
//}

int main(int argc, char ** argv)
{
	atexit(servo_exit);

	ros::init(argc, argv, "servo_pwm_control");
	ros::NodeHandle nh_;
	ros::Subscriber servo_sub = nh_.subscribe("pan_tilt", 100, servoCallback);

	std::string module_dir = "/home/root/pwm.ko";
//	nh_.getParam("moduledir", module_dir);
//	std::cout << module_dir << std::endl;
	ROS_INFO("Load Module %s", module_dir.c_str());
	int servo_max = 0, servo_min = 0;
	nh_.getParam("servo_max", servo_max);
	nh_.getParam("servo_min", servo_min);
	ROS_INFO("Servo Max: %d", servo_max);
	ROS_INFO("Servo Min: %d", servo_min);

	char command[100];
	sprintf(command, 
					"/sbin/insmod %s timers=9,10 servo=1 servo_min=%d servo_max=%d",
					module_dir.c_str(), servo_min, servo_max);

	setuid(0);
	system(command);
	std::cout << command << std::endl;

	ros::spin();

	return 0;

}
