#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <location_monitor/LandmarkDistance.h>

void landmarkCallback(const location_monitor::LandmarkDistance msg)
{
  if (msg.name.c_str() == "PickupLocation" || msg.name.c_str() == "DropoffLocation" && msg.distance > 0.75)
  {
    ros::init(argc, argv, "add_marker");
    ros::NodeHandle n;
    ros::Rate r(0.2);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    int shape = visualization_msgs::Marker::CUBE;

    while (ros::ok())
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "add_marker";
      marker.id = 0;
      marker.type = shape;
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
    }
  }
 
  else if (msg.name.c_str() == "PickupLocation" && msg.distance <= 0.75)
  {
    ROS_INFO("I'm near: [%s], Distance: [%f]", msg.name.c_str(), msg.distance);
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 0.0;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
    sleep(1);
  }

  else if (msg.name.c_str() == "DropoffLocation" && msg.distance <= 0.75)
  {
    ROS_INFO("I'm near: [%s], Distance: [%f]", msg.name.c_str(), msg.distance);
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
      ROS_WARN_ONCE("Please subscribe to the marker");
      sleep(1);
    }
    marker_pub.publish(dropoff);
  } 
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_marker");
  ros::NodeHandle n;
  ros::Rate r(0.2);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  int shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "add_marker";
    marker.id = 0;
    marker.type = shape;
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

    ros::Subscriber sub = n.subscribe("closest_landmark", 10, landmarkCallback);

    r.sleep();
  }
}
