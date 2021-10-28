# move_bot_catkin_package
A catkin package to move any urdf loaded on rviz using joint_state_publisher and MoveIt

### Getting Started
1. Create a catkin workspace for these packages to download.
2. Download the three packages in the src folder of the workspace.
3. Run `catkin_make` in the workspace folder.

### Description of Packages
#### 1.OW_arm42v3
Description of the OW_amr43 robotic arm.
The package was provided.
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
(NOTE: This code is currently not in a working condition and needs refinement. The complete version of this code will be uploaded in another branch upon completion.)

(Expected) Usage:
1. Launch the MoveIt model on rviz
```shell
roslaunch OW_arm42_moveit demo.launch
```
2. (In another terminal) Run the move_in_straight_line
```shell
rosrun move_bot move_in_straight_line
```
Follow the prompt.
