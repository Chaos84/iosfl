iosfl
===============

`iosfl` is a ROS package which implements the Input/Output State Feedback Linearization for a differential drive mobile robot (for reference, see http://www.dis.uniroma1.it/~labrob/pub/papers/CST02.pdf). 

1. Installation
	```bash
	cd path_to_your_catkin_workspace/src
	git clone https://github.com/Chaos84/iosfl.git
	```

2. Compile
	```bash
	cd path_to_your_catkin_workspace
	catkin_make
	```
3. Run
	```bash
	roscore
	open another tab
	rosrun iosfl iosfl_node
	```
	
4. Documentation
	Generate the documentation by running
	```bash
	roscd iosfl
	rosdoc_lite . -o ./doc
	```

By default, the node subscribes to the topic "/speed" and publishes the topic "/cmd_vel". Both topics are geometry_msgs/Twist type. The un-constrained desired speeds should be put in the fields var_name.linear.x and var_name.linear.y on the topic speed. The published topic has the linear speed in pub_msg.linear.x and the angular speed in pub_msg.angular.y .
By default, the lead the robot is contrained to has length equal to 0.1m, the maximum speed (per wheel) is 0.5m/s and the axle of the robot is 0.33m. You can change these parameters, for example, by running the node with a launch file:

	```xml
	<!-- iosfl Launch File -->
	<launch>
		<node pkg="iosfl" name="iosfl_node" type="iosfl_node">
			<param name="iosfl_lead" value="0.1" type="double"/>
			<param name="axle" value="0.33" type="double"/>
			<param name="maximum_speed" value="0.5" type="double"/>			
		</node>
	</launch>
	```
In order to obtain the yaw angle, the node computes the transformation between the frame "/map" and the frame "/base_link", so be sure to publish them correctly.

============
There is also a static method to saturate the robot speed, which can be used without creating an instance of the class iosfl. You can do it by adding the dependencies to the package "iosfl" in your package.xml and CMakeLists.txt files or by adding the dependency when you create your package i.e.:

	```bash
	cd path_to_your_catkin_workspace
	catkin_create_pkg your_package_name roscpp iosfl all_your_other_dependencies
	```

### THANKS!
Please let me know if something doesnt work, or if you have suggestions (or feel free to add stuff and send a pull request).
