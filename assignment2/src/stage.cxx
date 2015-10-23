
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <cstring>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

using namespace std;

int main(int argc, char **argv){

	ros::init(argc, argv, "stage");
	ros::NodeHandle nh_;
	ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	geometry_msgs::Twist base_cmd;
	string input;
	int startx,starty;
	int goalx,goaly;

	cout << "What is the intial location on a map x,y \n";
	cin >> startx >> starty;

	cout << "What is the goal location on a map x,y \n"
	cin >> goalx >> goaly;

	ros::Rate loop_rate(10);

	while(ros::ok()){

		

		cmd_vel_pub_.publish(base_cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
