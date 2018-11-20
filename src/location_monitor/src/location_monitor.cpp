#include "math.h"
#include <vector>
#include <string>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "location_monitor/LandmarkDistance.h"

using std::vector;
using std::string;
using location_monitor::LandmarkDistance;

class Landmark {
  public:
    Landmark(string name, double x, double y)
        : name(name), x(x), y(y) {}
    string name;
    double x;
    double y;
};

class LandmarkMonitor {
  public:
    LandmarkMonitor(const ros::Publisher& landmark_pub)
        : landmarks_(), landmark_pub_(landmark_pub) {
      InitLandmarks();
    }


    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;
      LandmarkDistance ld = FindClosest(x, y);
      landmark_pub_.publish(ld);

      ROS_INFO("I'm [%f] from the %s", ld.distance, ld.name.c_str());
      
    }

  private:
    vector<Landmark> landmarks_;
    ros::Publisher landmark_pub_;

    LandmarkDistance FindClosest(double x, double y) {
      LandmarkDistance result;
      result.distance = -1;

      for (size_t i = 0; i < landmarks_.size(); ++i) {
        const Landmark& landmark = landmarks_[i];
	double xd = landmark.x - x;
	double yd = landmark.y - y;
	double distance = sqrt(xd*xd + yd*yd);

	if (result.distance < 0 || distance < result.distance) {
	  result.name = landmark.name;
	  result.distance = distance;
	}
      }

      return result; 
    }

    void InitLandmarks() {
      landmarks_.push_back(Landmark("PickupLocation", -2.13, -1.77));
      landmarks_.push_back(Landmark("DropoffLocation", -4.42, 5.16));
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "location_monitor");
  ros::NodeHandle nh;
  ros::Publisher landmark_pub = nh.advertise<LandmarkDistance>("closest_landmark", 10);
  LandmarkMonitor monitor(landmark_pub); 
  ros::Subscriber sub = nh.subscribe("odom", 10, &LandmarkMonitor::OdomCallback, &monitor);
  ros::spin();
  return 0;
}
