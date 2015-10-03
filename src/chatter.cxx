#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int state=0;
std_msgs::String recieve_msg;

int swap(std::string stuff){
	if(stuff=="start"){
		return 1;
	}else if(stuff=="stop"){
		return 2;
	}else if(stuff=="pause"){
		return 3;
	}else if(stuff==""){
		return 0;
	}else{
		return -1;
	}
}

void controlCallback(const std_msgs::String::ConstPtr msg) {
	ROS_INFO("I received: [%s]", msg->data.c_str());
	recieve_msg.data=msg->data;
	state=swap(msg->data);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Subscriber control_sub = n.subscribe<std_msgs::String>("control_messages", 1000, controlCallback);
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(1);
	int count = 0;
	while (ros::ok()) {
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		switch(state){
			case 0:
				ROS_INFO("NO Message has been received!");
				break;
			case 1:
				chatter_pub.publish(msg);
				++count;
				ROS_INFO("Starting Publishing with command: %s",recieve_msg.data.c_str());
			break;
			case 2:
				count=0;
				ROS_INFO("Stopped Publishing with command: %s",recieve_msg.data.c_str());
			break;
			case 3:
				ROS_INFO("Paused Publishing with command: %s",recieve_msg.data.c_str());
			break;
			default:
				ROS_INFO("Something terrible happened with command: %s",recieve_msg.data.c_str());
		}
		ROS_INFO("%s", msg.data.c_str());
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

