#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <sstream>
#include <string>



bool chatter(assignment1::Messager::Request  &req,assignment1::Messager::Response &res){
  res.chatter=req.reqchatter;
  ROS_INFO("%s", res.chatter.c_str());
  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "talker_server");
  ros::NodeHandle n;
  ros::ServiceServer chatservice = n.advertiseService("chatter", chatter);
  ROS_INFO("Starting Talker Server!");
  ros::spin();

  return 0;
}