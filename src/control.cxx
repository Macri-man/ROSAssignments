#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv) {
	ros::init(argc, argv, "control_messages");
	ros::NodeHandle n;
	ros::Publisher control_pub = n.advertise<std_msgs::String>("control_messages", 1000);
	ros::Rate loop_rate(1);
	int count = 0;
	while (ros::ok()) {
		std_msgs::String msg;
		std::string input;
		std::cout << "Enter Command: ";
		std::cin >> msg.data;
		if(!(msg.data == "start" || msg.data == "stop" ||  msg.data =="pause")){
			ROS_INFO("Wrong Input: %s", msg.data.c_str());
		}else{
			ROS_INFO("Command Sent: %s", msg.data.c_str());
			control_pub.publish(msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}