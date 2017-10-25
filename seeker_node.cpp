///////////////////////////////////////////////////////////////////////////////
// Name:        Chris Evers
//
// Project:     TORCRobotics Interview Mini-Project
//
// Description: This is ROS node written in C++. It defines the behavior
//              for a seeker turtlebot. The turtlebot will seek out a ball
//              in a simulated environment and drive into it. It can be 
//              enabled and disabled with a rosservice call. It also publishes
//              relative distance to the ball on the /displacement topic.
//
// Date Last Updated: 25 October 2017
//

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

///////////////////////////////////////////////////////////////////////////
// This is the Seeker class we will be using to instatiate our turtlebot //
///////////////////////////////////////////////////////////////////////////
class Seeker
{
private:
  // The node handler we will be using
  ros::NodeHandle nh_;
  // We will be using an /enable service to start the scanning
  ros::ServiceServer service_;
  // We will be subscribing to the /scan topic to get 2D cross-sectional scans
  ros::Subscriber scan_sub_;
  // We will be publishing to the /mobile_base/commands/velocity topic to move the turtle
  ros::Publisher vel_pub_;
  // We will be publishing to the /displacement topic to get the relative displacement
  ros::Publisher disp_pub_;

  bool enabled_;        // Data field for enabling the Seeker
  bool found_;          // Data field to see if object is found
  float displacement_;  // Data field to contain relative distance to object

public:
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  // ROS node initialization
  Seeker(ros::NodeHandle &nh) {
    nh_ = nh;
    // Setup the enable service
    service_ = nh_.advertiseService("/enable", &Seeker::enable, this);
    // Setup the subcriber for the /scan topic
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Seeker::processLaserScan, this);
    // Setup the publisher for the /mobile_base/commands/velocity topic
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    // Setup the publisher for /displacement topic
    disp_pub_ = nh_.advertise<geometry_msgs::Vector3>("/displacement", 1000);

    displacement_ = nanf(""); // Initialize displacement to nan
    enabled_ = false;         // Initialize Seeker to be disabled
    found_ = false;           // Initialize ball not found

    ROS_INFO("ready to enable");
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // This is the callback function for the enable service. It enables and disables the Seeker
  bool enable(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    // Set response and enabled_ field depending on request
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

  //////////////////////////////////////////////////////////////////////////////////////////////
  // This is the callback function that process LaserScan data off the /scan topic
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan) {
    float min_distance = 10.0; // Initialize variable for min_distance
    //Iterate through ranges vector and test to see if ball is found
    for (int i = 150; i <= 490; i++) {
      // Ball is found if the first and last elements are nan and some middle elements are not nan
      if (isnan(scan->ranges.front()) && !isnan(scan->ranges[i]) && isnan(scan->ranges.back())) {
        // If found, calculate relative distance by finding least range value
        for (int i = 0; i < scan->ranges.size(); i++) {
          if (scan->ranges[i] < min_distance) {
            displacement_ = scan->ranges[i];
          }
        }
        found_ = true;
        break; // Break out of loop once ball is found
      }
      // Else, ball is not found and displacement is nan
      else {
        found_ = false;
        displacement_ = nanf("");
      }
    }
  }

  //////////////////////////////////////////////////////
  // This function publishes velocities for drive state
  void drive() {
    geometry_msgs::Twist msg;

    msg.linear.x = 0.6;

    vel_pub_.publish(msg);
  }

  /////////////////////////////////////////////////////
  // This function publishes velocities for seek state
  void seek() {
    geometry_msgs::Twist msg;

    msg.angular.z = 0.4;

    vel_pub_.publish(msg);
  }

  ///////////////////////////////////////////////////////////
  // This function publishes distance to /displacement topic
  // Displacement is only shown in the x direction
  void publishDisp() {
    geometry_msgs::Vector3 msg;

    msg.x = displacement_;
    msg.y = nanf("");
    msg.z = nanf("");

    disp_pub_.publish(msg);
  }

  //////////////////////////////////////////////////
  // This function returns the enabled_ data field
  bool isEnabled() {
    return enabled_;
  }

  ////////////////////////////////////////////////
  // This function returns the found_ data field
  bool ballFound() {
    return found_;
  }
};

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Init ROS, name node seeker_node
  ros::init(argc, argv, "seeker_node");
  
  // Declare node handler and contruct turtlebot object from Seeker
  ros::NodeHandle n;
  Seeker turtlebot(n);

  // Set loop rate to 60Hz
  ros::Rate loop_rate(60);

  // Run this while ROS isn't shutdown
  while (ros::ok()) {
    // Wait for turtlebot to be enabled
    while (!turtlebot.isEnabled()) {
      ros::spinOnce();
    }

    // Once enabled run this
    while(turtlebot.isEnabled()) {
      // Drive into ball if found
      if (turtlebot.ballFound()) {
        turtlebot.drive();
      }
      // If not found, seek ball
      else {
        turtlebot.seek();
      }
      turtlebot.publishDisp(); // Publish displacement
      ros::spinOnce();         // Process callbacks once per iteration
      loop_rate.sleep();       // Wait 1/60 sec before continuing
    }    
  }

  return 0;
}
