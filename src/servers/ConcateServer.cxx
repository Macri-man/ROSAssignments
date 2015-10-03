#include "ros/ros.h"
#include "assignment1/Concate.h"
#include <sstream>
#include <string>

std::stringstream ss;

int count=0;


bool concate(assignment1::Concate::Request  &req,assignment1::Concate::Response &res){
  if(count%5==0){
      req.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());
      concate_pub.publish(msg);
      ss.str(std::string());
    }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Concate");
  ros::NodeHandle n;
  assignment1::Concate::Request  &req;
  ros::ServiceServer service = n.advertiseService("Concate", concate);

  return 0;
}