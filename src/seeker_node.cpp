#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/SetBool.h"
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>

class Seeker
{
private:
  // The node handler we'll be using
  ros::NodeHandle nh_;
  // We will be publishing to the /mobile_base/commands/velocity topic to move the turtle
  ros::Publisher cmd_vel_pub_;
  // We will be publishing to the /displacement topic to get the relative displacement
  ros::Publisher disp_pub_;
  // We will be subscribing to the /scan topic to get 2D cross-sectional scans
  ros::Subscriber scan_sub_;
  // We will be using an /enable service to start the scanning
  ros::ServiceServer service_;

  bool enabled_;
  bool found_;

public:
  // ROS node initialization
  Seeker(ros::NodeHandle &nh) {
    nh_ = nh;
    // set up the publisher for the /mobile_base/commands/velocity topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    // set up the publisher for /displacement topic
    disp_pub_ = nh_.advertise<geometry_msgs::Vector3>("/displacement", 1000);
    // set up the subcriber for the /scan topic
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Seeker::processLaserScan, this);
    // setup the service
    service_ = nh_.advertiseService("/enable", &Seeker::enable, this);
    ROS_INFO("ready to enable");

    enabled_ = false;

    found_ = false;
  }

  bool enable(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if (req.data) {
      res.success = true;
      enabled_ = true;
      res.message = "enabling seeker";
      ROS_INFO("seeker enabled");
    }
    else {
      res.success = true;
      enabled_ = false;
      res.message = "disabling seeker";
      ROS_INFO("seeker disabled");
    }

    return true;
  }

  bool isEnabled() {
    return enabled_;
  }

  bool ballFound() {
    return found_;
  }

  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan) {
    for (int i = 0; i < scan->ranges.size(); i++) {
      if (!isnan(scan->ranges[i]) && isnan(scan->ranges.front()) && isnan(scan->ranges.back())) {
        found_ = true;
        ROS_INFO("ball found");
        break;
      }
      else {
        found_ = false;
      }
    }
  }

  void search() {
    geometry_msgs::Twist msg;

    geometry_msgs::Vector3 l;
    geometry_msgs::Vector3 a;

    l.x = 0.0;
    l.y = 0.0;
    l.z = 0.0;

    a.x = 0.0;
    a.y = 0.0;
    a.z = 0.8;

    msg.linear = l;
    msg.angular = a;

    cmd_vel_pub_.publish(msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seeker_node");
  
  ros::NodeHandle n;
  Seeker turtlebot(n);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    while (!turtlebot.isEnabled()) {
      ros::spinOnce();
    }

    while(turtlebot.isEnabled()) {
      turtlebot.search();
      ros::spinOnce();
      loop_rate.sleep();  
    }    
  }

  return 0;
}
