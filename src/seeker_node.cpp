#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>

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
  //ros::ServiceServer enable_;
  //ros::ServiceClient client_;

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
    // up the service
    //client_ = nh_.serviceClient<seeker::enable>("enable");
  }

  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    //scan->ranges[] are laser readings
    //...
  }

  // Scan for the ball here and returns if found
  geometry_msgs::Vector3 seekBall() {
    bool found = false;
    geometry_msgs::Vector3 displacement;

    while(!found) {

      if (1) {
        found = true;
      }
    }

    return displacement;
  }

  void move() {
    geometry_msgs::Twist msg;

    geometry_msgs::Vector3 l;
    geometry_msgs::Vector3 a;

    l.x = 2.0;
    l.y = 0.0;
    l.z = 0.0;

    a.x = 0.0;
    a.y = 0.0;
    a.z = 0.8;

    msg.linear = l;
    msg.angular = a;

    cmd_vel_pub_.publish(msg);
  }
  // Publish to the /mobile_base/commands/velocity topic here and ram the ball
  // based on displacement
  void ramBall(const geometry_msgs::Vector3& location) {

    return;
  }

  // Write the /enable service server here
  void enable() {

  }
};

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "seeker_node");
  
  ros::NodeHandle n;
  Seeker turtlebot(n);

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    turtlebot.move();
    ros::spinOnce();
    loop_rate.sleep();
  }
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  /*ros::Publisher displacement_pub = n.advertise<geometry_msgs::Vector3>("displacement", 1000);
  ros::Publisher 

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    ++count;
  }


  return 0;
}
