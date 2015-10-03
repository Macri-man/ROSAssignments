#include "ros/ros.h"
#include "assignment1/Messages.h"
#include <sstream>
#include <string>

int count=0;
int chatcount=0;

bool concate(assignment1::Messages::Request  &req,assignment1::Messages::Response &res){
  std::stringstream ss;
  ss << req.concate << " ";
  if(count%5==0){
    ROS_INFO("Concate: %s", req.concate.c_str());
    ss.str(std::string());
  }
  count++;
  return true;
}

bool command(assignment1::Messages::Request  &req,assignment1::Messages::Response &res){
    if(!(req.command == "start" || req.command == "stop" ||  req.command =="pause")){
      ROS_INFO("Wrong Input: %s", req.command.c_str());
    }else{
      ROS_INFO("Command Sent: %s", req.command.c_str());
      res.response=req.command;
    }
  return true;
}

bool chat(assignment1::Chatter::Request  &req,assignment1::Chatter::Response &res){
  std::stringstream ss;
  ss << req.chatter << ++chatcount;
  res.reschatter=ss.str();
  ROS_INFO("request: %s", req.A);
  ROS_INFO("sending back response: [%s]", res.B);
  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "Message");
  ros::NodeHandle n;
  ros::ServiceServer conservice = n.advertiseService("Concate", concate);
  ros::ServiceServer chatservice = n.advertiseService("Chatter", chatter);
  ros::ServiceServer comservice = n.advertiseService("Command", command);
  ROS_INFO("Starting Server!");
  ros::spin();

  return 0;
}