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


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_marker");
  ros::NodeHandle n;
  Listener listener;
  ros::Rate loop_rate(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("closest_landmark", 10, &Listener::LocationCallback, &listener);

  string location = listener.name;
  float dist = listener.distance;
  ros::spinOnce();

  while (ros::ok())
  {

    ROS_INFO("Location: [%s], Distance: [%f]", listener.name.c_str(), listener.distance);

//    if (listener.name.c_str() == "PickupLocation" && listener.distance > 1.0 || listener.name.c_str() == "DropoffLocation" && listener.distance > 1.0)
 //   {
//    ROS_INFO("Travelling");
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "add_marker";
    marker.id = 0; 
    marker.type = visualization_msgs::Marker::CUBE; 
    marker.action = visualization_msgs::Marker::ADD;  
    marker.pose.position.x = -2.131610;
    marker.pose.position.y = -1.770400;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;  
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0; 
    marker.lifetime = ros::Duration(); 
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    ros::spinOnce();
    loop_rate.sleep();

    if (location.c_str() == "PickupLocation" && listener.distance <= 1.0) 
    {
      ROS_INFO("Stage 1");
//      visualization_msgs::Marker marker;
//      marker.header.frame_id = "/map";
//      marker.header.stamp = ros::Time::now();
//      marker.ns = "add_marker";
//      marker.id = 0; 
//      marker.type = visualization_msgs::Marker::CUBE; 
      marker.action = visualization_msgs::Marker::ADD;  
      marker.pose.position.x = -2.131610;
      marker.pose.position.y = -1.770400;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.25;
      marker.scale.y = 0.25;
      marker.scale.z = 0.25;  
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.0; 
      marker.lifetime = ros::Duration(); 
      while (marker_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }
      marker_pub.publish(marker);
      ros::spinOnce();
      loop_rate.sleep();
    }

    else if (location.c_str() == "DropoffLocation" && listener.distance <= 1.0)
    {
      ROS_INFO("Stage 2");
      visualization_msgs::Marker dropoff;
      dropoff.header.frame_id = "/map";
      dropoff.header.stamp = ros::Time::now();
      dropoff.ns = "add_marker";
      dropoff.id = 1;
      dropoff.type = visualization_msgs::Marker::CUBE;
      dropoff.action = visualization_msgs::Marker::ADD;
      dropoff.pose.position.x = -4.419000;
      dropoff.pose.position.y = 5.158160;
      dropoff.pose.position.z = 0;
      dropoff.pose.orientation.x = 0.0;
      dropoff.pose.orientation.y = 0.0;
      dropoff.pose.orientation.z = 0.0;
      dropoff.pose.orientation.w = 1.0;
      dropoff.scale.x = 0.25;
      dropoff.scale.y = 0.25;
      dropoff.scale.z = 0.25;
      dropoff.color.r = 0.0f;
      dropoff.color.g = 1.0f;
      dropoff.color.b = 0.0f;
      dropoff.color.a = 1.0;
      dropoff.lifetime = ros::Duration();
      while (marker_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }

      marker_pub.publish(dropoff);
      ros::spinOnce();
      break;
    }
    else
    {
      ROS_INFO("Last exit");
    }
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0; 
}
