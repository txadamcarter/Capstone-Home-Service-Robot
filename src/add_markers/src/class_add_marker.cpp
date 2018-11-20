#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <location_monitor/LandmarkDistance.h>
#include <string.h>
using namespace std;

class Listener
{
  public:
    string name;
    float distance;
    void LocationCallback(const location_monitor::LandmarkDistance::ConstPtr& msg);
};

void Listener::LocationCallback(const location_monitor::LandmarkDistance::ConstPtr& msg)
{
  name = msg->name.c_str();
  distance = msg->distance; 
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "add_marker");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  Listener listener;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("closest_landmark", 10, &Listener::LocationCallback, &listener);
  
  while (ros::ok())
  {
    ros::spinOnce();
    ROS_INFO("Nearest: [%s], Distance: [%f]", listener.name.c_str(), listener.distance);
    loop_rate.sleep();
  }
//  ros::spin();
  return 0;
}
