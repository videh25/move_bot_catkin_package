# move_bot_catkin_package
A catkin package to move any urdf loaded on rviz using joint_state_publisher and MoveIt

### Getting Started
1. Create a catkin workspace for these packages to download.
2. Download the three packages in the src folder of the workspace.
3. Run `catkin_make` in the workspace folder.

### Description of Packages
#### 1.OW_arm42v3
Description of the OW_amr43 robotic arm.
The package was provided. (NOTE: Minor changes were made to the package to import it properly in MoveIt.)
#### 2. OW_arm42_moveit
The OW_arm42v3 package imported on MoveIt.
The package was created using moveit's setup assisstant.
#### 3. move_bot
Package developed to control the model imported on rviz.


### 3 main functions:
#### 1. terminal_joint_state_publisher.py
A node that uses joint_state_publisher to take values from terminal and adjust the joint variables according for ANY urdf model imported to rviz.

Usage:
1. Launch the model on rviz (NOTE: No joint_state_publisher is initialised yet.)
```shell
roslaunch OW_arm42v3 display.launch
```
2. (In another terminal) Run the terminal_joint_state_publisher.py
```shell
rosrun move_bot terminal_joint_state_publisher.py
```
Follow the prompt. (NOTE: You can pass all values of velocity and effort as zero for convinience.)

#### 2. control_joint_state_moveit
A node to control the model import on MoveIt to achieve the required position.

Usage:
1. Launch the MoveIt model on rviz
```shell
roslaunch OW_arm42_moveit demo.launch
```

2. (In another terminal) Run the control_joint_state_moveit
```shell
rosrun move_bot control_joint_state_moveit
```
Follow the prompt. (NOTE: You can pass all values of velocity and effort as zero for convinience.)

#### 3. move_in_straight_line
A node to control the model import on MoveIt to achieve the required position ONLY through **straight lines**.
(**NOTE**: The kinematic constraints of the imported model will allow only specific paths to be followed. Also, a straight path is followed from current position to the starting position too, which may happen to lie outside the workspace or at null points of the manipulator. Thus, it is advised to first set the position of arm at some point closer to the starting point using _control_joint_state_moveit_ and then use _move_in_straight_line_.)

Usage:
1. Launch the MoveIt model on rviz
```shell
roslaunch OW_arm42_moveit demo.launch
```
2. (In another terminal) Run the move_in_straight_line
```shell
rosrun move_bot move_in_straight_line
```
Follow the prompt.

## Videos
### 1. Video showing the usage of terminal_joint_state_publisher.py (Controlled the imported urdf on rviz)
https://user-images.githubusercontent.com/66770479/139361968-4cfb3c29-b7dc-448b-b55d-0e41d13838ae.mp4

### 2. Video showing usage of control_joint_state_moveit and move_in_straight_line (Controlled the generated MoveIt package on rviz)
https://user-images.githubusercontent.com/66770479/139361947-37f0cc80-018a-45a2-b8e8-63f434c955e5.mp4




## Sources refered extensively while creating the package:
1. http://wiki.ros.org/joint_state_publisher
2. https://github.com/ros/joint_state_publisher
3. https://gramaziokohler.github.io/compas_fab/latest/examples/03_backends_ros/08_ros_create_moveit_package_from_custom_urdf.html
4. http://wiki.ros.org/joint_state_publisher
5. https://gramaziokohler.github.io/compas_fab/latest/examples/03_backends_ros/08_ros_create_moveit_package_from_custom_urdf.html
6. https://github.com/PickNikRobotics/rviz_visual_tools
