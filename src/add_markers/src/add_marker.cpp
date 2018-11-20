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
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  Listener listener;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("closest_landmark", 10, &Listener::LocationCallback, &listener);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    Listener listener;
//    ros::spinOnce();
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "marker_0";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -2.131610;
    marker.pose.position.y = -1.770400;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(15.0);  // specify for how long this marker should stay before being deleted
    ROS_INFO("Publishing marker at pickup location");

    // Publish the marker
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
    ROS_INFO("Location: [%s], Distance: [%f]", listener.name.c_str(), listener.distance);
    ros::spinOnce();
    // wait for 5 seconds and then delete the marker
    ROS_INFO("Waiting for bot arrival");
    ros::Duration(15.0).sleep();

    ROS_INFO("Deleting the marker");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    //wait for 5 seconds to delete the marker
    ROS_INFO("Waiting for delivery");
    ros::Duration(30.0).sleep();

    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -4.419000;
    marker.pose.position.y = 5.158160;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.5;
    marker.lifetime = ros::Duration();
    // publish marker at drop off zone
    ROS_INFO("Publishing marker at dropoff location");
    marker_pub.publish(marker);
    ros::spin();
    return 0;
  }

}
