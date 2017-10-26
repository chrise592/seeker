       ROS Kinetic Package: seeker

This ROS package contains one node: seeker_node.

Steps to run:

1. Have Ubuntu 16.04, ROS, and Turtlebot Gazebo intalled.

2. Clone this repository into a catkin_ws/src/ directory.

3. Build package using the command in the catkin_ws directory: 
	catkin_make

4. Start the simulation world using the following command:
	roslaunch turtlebot_gazebo turtlebot_world.launch world_file:='"<path>/mini_world.world"'
	
   Note: You can also include any world, and place your own objects.

5. Run the seeker_node using this command:
	rosrun seeker seeker_node
		or
	roslaunch seeker interview.launch

6. Enable the turtlebot to seek out the ball:
	rosservice call /enable "data: true"

7. Watch the turtlebot seek out and drive into the ball.

8. Disable the turtlebot:
	rosservice call /enable "data: false"

9. You can see the displacement data by running this command:
	rostopic show displacement

   Note: Displacement is only published after the turtlebot is enabled. Displacement is only shown in the x direction as relative distance to the ball. x is nan when the ball is not in range of the sensor or not found. y and z remain nan throughout.
	
Possible future additions:

	Implement a feedback loop for intended direction and rotational velocity to make more precise movements and prevent overshooting directions. It is also possible to adjust angular velocity depeding on the position of the ball in the laser scan. (i.e. Rotate left if ball is off-center left, rotate right if ball is off-center right)

	Stop the robot upon bumping into the ball.

