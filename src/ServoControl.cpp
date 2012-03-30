#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <string>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

FILE *pwm9, *pwm10;

void servoCallback(const std_msgs::String::ConstPtr& msg) {
	//std::cout << msg->data << std::endl;
//	char *pch = ;msg->data.substr(0,5).c_str()
//	const char *tok = ",";
//	pch = strtok(const_cast<char *>(msg->data.c_str()), tok);
	std::string p9 = msg->data.substr(0,5);
	int res = fprintf(pwm9, "%s", p9.c_str());
	if (res != 0) {
		std::cout << "PWM9 Error " << res << std::endl;
	}

//	pch = strtok(NULL,",");
//	pch = msg->data.sub
	std::string p10 = msg->data.substr(6,5);
	res = fprintf(pwm10, "%s", p10.c_str());
	if (res != 0) {
		std::cout << "PWM10 Error " << res << std::endl;
	}
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
	ros::init(argc, argv, "servo_pwm_control");
	ros::NodeHandle nh_;
	ros::Subscriber servo_sub = nh_.subscribe("pan_tilt", 100, servoCallback);

	pwm9 = fopen ("/dev/pwm9","w");
	pwm10 = fopen("/dev/pwm10","w");
	
	ros::spin();
	

	fclose(pwm9);
	fclose(pwm10);

	return 0;

}
