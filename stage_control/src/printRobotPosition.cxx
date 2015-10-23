#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;
string tf_prefix;

pair<double, double> getRobotPosition();

int main(int argc, char** argv) {
    ros::init(argc, argv, "print_position");
    ros::NodeHandle nh;
    
    // Get tf_prefix from the parameter server
    nh.getParam("tf_prefix", tf_prefix);
    pair<double, double> currPosition;
    ros::Rate loopRate(1);

    while (ros::ok()) {
        currPosition = getRobotPosition();
        ROS_INFO("Current pose: (%.3f, %.3f)", currPosition.first, currPosition.second);
        loopRate.sleep();
    }
    
    return 0;
}

pair<double, double> getRobotPosition()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    pair<double, double> currPosition;

    try {
        string base_footprint_frame = tf::resolve(tf_prefix, "base_footprint");
	
        listener.waitForTransform("/odom", base_footprint_frame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/odom", base_footprint_frame, ros::Time(0), transform);

        currPosition.first = transform.getOrigin().x();
        currPosition.second = transform.getOrigin().y();
    }
    catch (tf::TransformException &ex) {
ROS_ERROR("%s",ex.what());
    }
    return currPosition;
}

