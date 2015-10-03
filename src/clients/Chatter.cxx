#include "ros/ros.h"
#include "assignment1/Chatter.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Chatter");
  int count =0;
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<assignment1::Chatter>("Chatter");
  assignment1::Chatter srv;
  std_msgs::String msg;
  std::stringstream ss;
  while (ros::ok()) {
    ss << "hello world " << count;
    srv.request.A = ss.str();
    if (client.call(chattersrv)){
      ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }else{
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }
    switch(state){
      case 0:
        ROS_INFO("NO Message has been received!");
        break;
      case 1:
        chatter_pub.publish(msg);
        ++count;
        ROS_INFO("Starting Publishing with command: %s",recieve_msg.data.c_str());
      break;
      case 2:
        count=0;
        ROS_INFO("Stopped Publishing with command: %s",recieve_msg.data.c_str());
      break;
      case 3:
        ROS_INFO("Paused Publishing with command: %s",recieve_msg.data.c_str());
      break;
      default:
        ROS_INFO("Something terrible happened with command: %s",recieve_msg.data.c_str());
    }
    ROS_INFO("%s", msg.data.c_str());
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}