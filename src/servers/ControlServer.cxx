#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <sstream>
#include <string>
#include <cstdlib>

int swap(std::string command){
  if(command=="start"){
    return 1;
  }else if(command=="stop"){
    return 2;
  }else if(command=="pause"){
    return 3;
  }else if(command==" "){
    return 4;
  }else{
    return -1;
  }
}

bool control(assignment1::Messager::Request  &req,assignment1::Messager::Response &res){
    std::cout << "Enter Command: ";     
    std::cin >> res.command;
    res.state=swap(res.command);
    ROS_INFO("Command: [%s]",res.command.c_str());
    if(res.command==""){
      ROS_INFO("No Command has been received!");
    }else if(!(res.command == "start" || res.command == "stop" ||  res.command =="pause")){
      ROS_INFO("Wrong Input: [%s]", res.command.c_str());
    }
  return true;
}



int main(int argc, char **argv){

  ros::init(argc, argv, "control_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("control_messages", control);
  ROS_INFO("Starting Control Server!");
  ros::spin();

  return 0;
}