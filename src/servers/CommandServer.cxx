#include "ros/ros.h"
#include "assignment1/Command.h"

bool command(assignment1::Command::Request  &req,assignment1::Command::Response &res){
   std::cout << "Enter Command Master: ";
    std::cin >> req.command;
    if(!(req.command == "start" || req.command == "stop" ||  req.command =="pause")){
      ROS_INFO("Wrong Input: %s", req.command.c_str());
    }else{
      ROS_INFO("Command Sent: %s", req.command.c_str());
      res.response=req.command;
    }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Command");
  ros::NodeHandle n;
  assignment1::Command::Request  &req;
  ros::ServiceServer service = n.advertiseService("Command", command);

  return 0;
}