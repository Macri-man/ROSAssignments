#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

std_msgs::String received_msg;
std::stringstream ss;
ros::Publisher concate_pub;

int count = 0;

void concatesub(const std_msgs::String::ConstPtr msg) {
	
	if(msg->data.c_str()!=" " || msg->data.c_str()!=NULL){
		ROS_INFO("I received: [%s]", msg->data.c_str());
		ss << msg->data.c_str() << " ";
		++count;
		if(count%5==0){
			received_msg.data = ss.str();
			ROS_INFO("%s", received_msg.data.c_str());
			concate_pub.publish(received_msg);
			ss.str(std::string());
		}
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "concate");
	ros::NodeHandle n;
	ros::Subscriber concate_sub = n.subscribe<std_msgs::String>("chatter", 1000, concatesub);
	concate_pub = n.advertise<std_msgs::String>("concate", 1000);
	ros::spin();
	return 0;
}

