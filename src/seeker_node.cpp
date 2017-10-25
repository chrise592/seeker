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
  ros::Publisher velocity_pub_;
  // We will be publishing to the /displacement topic to get the relative displacement
  ros::Publisher disp_pub_;
  // We will be subscribing to the /scan topic to get 2D cross-sectional scans
  ros::Subscriber scan_sub_;
  // We will be using an /enable service to start the scanning
  ros::ServiceServer service_;

  bool enabled_;
  bool found_;
  float displacement_;

public:
  // ROS node initialization
  Seeker(ros::NodeHandle &nh) {
    nh_ = nh;
    // set up the publisher for the /mobile_base/commands/velocity topic
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    // set up the publisher for /displacement topic
    disp_pub_ = nh_.advertise<geometry_msgs::Vector3>("/displacement", 1000);
    // set up the subcriber for the /scan topic
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Seeker::processLaserScan, this);
    // setup the service
    service_ = nh_.advertiseService("/enable", &Seeker::enable, this);
    ROS_INFO("ready to enable");

    enabled_ = false;

    found_ = false;

    displacement_ = nanf("");
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
    float min_distance = 10.0;
    if (isnan(scan->ranges.front()) && isnan(scan->ranges.back())) {
      if (!isnan(scan->ranges[270]) || !isnan(scan->ranges[360]) || !isnan(scan->ranges[430])) {
        found_ = true;
        for (int i = 0; i < scan->ranges.size(); i++) {
          if (scan->ranges[i] < min_distance) {
            min_distance = scan->ranges[i];
          }
        }
        displacement_ = min_distance;
      }
    }
    else {
      found_ = false;
      displacement_ = nanf("");
    }
  }

  void seek() {
    geometry_msgs::Twist msg;

    msg.angular.z = 0.4;

    velocity_pub_.publish(msg);
  }

  void drive() {
    geometry_msgs::Twist msg;

    msg.linear.x = 0.4;

    velocity_pub_.publish(msg);
  }

  void publishDisp() {
    geometry_msgs::Vector3 msg;

    msg.x = displacement_;

    disp_pub_.publish(msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seeker_node");
  
  ros::NodeHandle n;
  Seeker turtlebot(n);

  ros::Rate loop_rate(60);

  while (ros::ok()) {
    while (!turtlebot.isEnabled()) {
      ros::spinOnce();
    }

    while(turtlebot.isEnabled()) {
      turtlebot.publishDisp();
      if (turtlebot.ballFound()) {
        turtlebot.drive();
      }
      else {
        turtlebot.seek();
      }
      ros::spinOnce();
      loop_rate.sleep();  
    }    
  }

  return 0;
}
