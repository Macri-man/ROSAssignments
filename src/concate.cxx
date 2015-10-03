#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

std_msgs::String msg;
std::stringstream ss;

int count = 0;

void concatesub(const std_msgs::String::ConstPtr msg) {
	
	if(msg->data.c_str()!=" " || msg->data.c_str()!=NULL){
		ROS_INFO("I found: [%s]", msg->data.c_str());
		ss << msg->data.c_str() << " ";
		++count;
	}else{
		ROS_INFO("No data recieved: [%s]", msg->data.c_str());
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "concate");
	ros::NodeHandle n;
	ros::Subscriber concate_sub = n.subscribe<std_msgs::String>("chatter", 1000, concatesub);
	ros::Publisher concate_pub = n.advertise<std_msgs::String>("concate", 1000);
	ros::Rate loop_rate(1);
	while (ros::ok()) {
		if(count%5==0){
			msg.data = ss.str();
			ROS_INFO("%s", msg.data.c_str());
			concate_pub.publish(msg);
			ss.str(std::string());
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

