#include "ros/ros.h"
#include "std_msgs/String.h"
#include "assignment1/Messager.h"
#include <sstream>
#include <iostream>
#include <string>

int state=0;
int service=0;
std_msgs::String recieve_msg;

int swap(std::string command){
	if(command=="start"){
		return 1;
	}else if(command=="stop"){
		return 2;
	}else if(command=="pause"){
		return 3;
	}else if(command==""){
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
	if (argc != 2){
      ROS_INFO("usage: talker_node service or talker_node publish");
      return 1;
    }
    if(std::string(argv[1])=="publish"){
    	service=2;
    }else{
    	service=1;
    }
    std::cerr << argv[1] << " " << service << "\n";
	ros::NodeHandle n;
	assignment1::Messager srv;
	ros::ServiceClient client;
	ros::Subscriber control_sub;
	if(service==2){
		control_sub = n.subscribe<std_msgs::String>("control_messages", 1000, controlCallback);
	}else{
		client = n.serviceClient<assignment1::Messager>("control_messages",false);
	}
	std::cerr << "Valid:" << client.isValid() << " Persistent:" << client.isPersistent() << " Name:" << client.getService();
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(1);
	int count = 0;
	while (ros::ok()) {
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		if(service==1){
			//std::cout << "Enter Command:";
			//std::getline (std::cin,recieve_msg.data);
			//srv.request.reqcommand=recieve_msg.data;
			if(client.call(srv)){
    			//ROS_INFO("Command: %s", srv.response.command.c_str());
    			state=srv.response.state;
    		}else{
    			ROS_ERROR("Failed to call service control_messages");
  			}
		}
		switch(state){
			case 0:
				ROS_INFO("NO Command has been received!");
				break;
			case 1:
				ROS_INFO("Starting Publishing with command: %s",recieve_msg.data.c_str());
				state=4;
			case 4:
				ROS_INFO("%s", msg.data.c_str());
				chatter_pub.publish(msg);
				++count;
			break;
			case 2:
				ROS_INFO("Stopped Publishing with command: %s",recieve_msg.data.c_str());
				state=5;
			case 5:
				ROS_INFO("%s", msg.data.c_str());
				count=0;
			break;
			case 3:
				ROS_INFO("Paused Publishing with command: %s",recieve_msg.data.c_str());
				state=6;
			case 6:
				ROS_INFO("%s", msg.data.c_str());
			break;
			default:
				ROS_INFO("Something terrible happened with command: %s",recieve_msg.data.c_str());
		}
		//ROS_INFO("%s", msg.data.c_str());
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

