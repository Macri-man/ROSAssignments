#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <sstream>
#include <string>

int count=0;

bool concate(assignment1::Messager::Request  &req,assignment1::Messager::Response &res){
  std::stringstream ss;
  ss << req.reqconcate << " ";
  if(count%5==0){
    res.concate=ss.str();
    ROS_INFO("%s", res.concate.c_str());
    ss.str(std::string());
  }
  ++count;
  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "concate_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("concate", concate);
  ROS_INFO("Starting Concate Server!");
  ros::spin();

  return 0;
}