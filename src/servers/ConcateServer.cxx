#include "ros/ros.h"
#include "assignment1/Concate.h"
#include <sstream>
#include <string>

std::stringstream ss;
int count=0;

bool concate(assignment1::Concate::Request  &req,assignment1::Concate::Response &res){
  ss << req.A << " ";
  if(count%5==0){
    ROS_INFO("Concate: %s", req.A.c_str());
    ss.str(std::string());
  }
  count++;
  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "Concate");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("Concate", concate);

  return 0;
}