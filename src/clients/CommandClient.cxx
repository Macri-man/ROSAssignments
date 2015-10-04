#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sstream>

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


int main(int argc, char **argv){
  ros::init(argc, argv, "Command");
  int count = 0;
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<assignment1::Messager>("Command");
  assignment1::Messager srv;
  ros::Rate loop_rate(1);
  std_msgs::String msg;
  std::stringstream ss;
  while (ros::ok()) {
	std::cout << "Enter Command: ";   	
	std::cin >> msg.data;
	srv.request.reqcommand=msg.data;
	srv.request.changestate=swap(msg.data);
	ROS_INFO("Sent Command to server: %s", srv.request.reqcommand.c_str());
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}