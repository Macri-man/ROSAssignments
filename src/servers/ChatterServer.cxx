#include "ros/ros.h"
#include "assignment1/Chatter.h"

bool chat(assignment1::Chatter::Request  &req,assignment1::Chatter::Response &res){
  res.B=req.A;
  ROS_INFO("request: %s", req.A);
  ROS_INFO("sending back response: [%s]", res.B);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "chatter");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("chatter", chat);
  ROS_INFO("Chatting");
  ros::spin();

  return 0;
}