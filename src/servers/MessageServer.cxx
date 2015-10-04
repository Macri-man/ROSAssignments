#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <sstream>
#include <string>

int count=0;
int chatcount=0;

bool concate(assignment1::Messager::Request  &req,assignment1::Messager::Response &res){
  std::stringstream ss;
  ss << req.reqchatter << " ";
  if(count%5==0){
    res.concate=ss.str();
    ROS_INFO("Concate: %s", res.concate.c_str());
    ss.str(std::string());
  }
  ++count;
  return true;
}

bool command(assignment1::Messager::Request  &req,assignment1::Messager::Response &res){
    if(!(req.reqcommand == "start" || req.reqcommand == "stop" ||  req.reqcommand =="pause")){
      ROS_INFO("Wrong Input: %s", req.reqcommand.c_str());
    }else{
      ROS_INFO("Command Sent: %s", req.reqcommand.c_str());
      res.command=req.reqcommand;
    }
  return true;
}

bool chatter(assignment1::Messager::Request  &req,assignment1::Messager::Response &res){
  res.chatter=req.reqchatter;
  ROS_INFO("request: %s", req.reqchatter.c_str());
  ROS_INFO("sending back response: [%s]", res.chatter.c_str());
  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "MessageServer");
  ros::NodeHandle n;
  ros::ServiceServer conservice = n.advertiseService("Concate", concate);
  ros::ServiceServer chatservice = n.advertiseService("Chatter", chatter);
  ros::ServiceServer comservice = n.advertiseService("Command", command);
  ROS_INFO("Starting Server!");
  ros::spin();

  return 0;
}