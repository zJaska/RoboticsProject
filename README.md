# ROS ~ Robotics course's first project
Politecnico di Milano - Accademic Year 2021-2022

>### Team
>* [__Elia Maggioni__](https://github.com/Eliaxie)
>* [__Flavio Renzi__](https://github.com/FlavioRenzi)
>* [__Jaskaran Singh__](https://github.com/zJaska)

The project was implemented utilizing `ROS Noetic` on a `Linux Ubuntu 20.04` machine.<br>
Tested also with `ROS Melodic` on a `Linux Ubuntu 18.04` machine.

## Goals
- Compute odometry using appropriate kinematics:
    - Compute robot linear and angular velocities v, ⍵ from wheel encoders.
    - Compute odometry using both Euler and Runge-Kutta integration.
    - ROS parameter for initial pose.
    - Calibrate (fine-tune) robot parameters to match ground truth.
- Compute wheel control speeds from v, ⍵.
- Add a service to reset the odometry to a specified pose (x,y,θ).
- Use dynamic reconfigure to select between integration method.

##
> To see the complete requirements -> [Project presentation](Project1.pdf)


## Package content
- bags: directory containing the supplied bags.
- cfg: directory containing the python file for dynamic reconfiguration of the integration type.
- launch: directory containing the launch-file that runs the necessary nodes and sets the initial parameters (initial coordinates and robot dimensions)
- msg: contains the file which defines the custom message publishing the wheel speed.
- src: contains the node code in c++.
- srv: directory containing the files to define the 2 custom services to set or reset the position in the odometry.
- rviz.rviz: config file to see the robot odometry.

## Nodes
![BlockDiagram](./BlockDiagram.drawio.svg)
- EncoderParser: reads the tick from the encoder and computes the speed of each wheel in rpm publishing it in the custom message on topic `/wheel_speed`.
- KinematicCore: reads the wheel speed and computes the kinematic of the robot in each axis publishing all in the topic `/cmd_vel`.
- OdometryCore: 
this node is used to compute the odometry of the robot, to publish all the related messages and to expose two services to set the position.
The node reads the messages published by KinematicCore node and, based on the value of the param "integration",
it computes the odometry of the robot using Euler or Runge-Kutta integration. After this it publishes the odometry on a topic and the tf transformation.
For the integration type, the node uses the dynamic reconfigure to set the integration type; the default value is "Euler"
and whenever the user changes the param a callback is executed, changing the value of the field "integrationType" of the class `Pub_Sub_Odometry_core`.
The node exposes also two callbacks for the two services: `resetZero`, that set the x and y positions to 0 when the user calls the service;
and `resetGeneral`, that set the pose of the robot equal to the value x, y and theta that the user passes when calls the service.
The node reads the initial pose of the robot from the params declared in the launch file and sets the x, y and theta of the robot.
- ReverseKinematicCore: reads the speed of the robot and calculates the speed at which each wheel needs to move to obtain that movement.



## Parameters
In the launch file there is the possibility to tune the parameters depending on the robot geometry:
- Width
- Length
- WheelRad
- CPM

and to set the initial position using the static transformation without having to recompile the code.<br>
There is also the parameter dimAvg that can be used to define the dimension of the moving average filter on the wheels speeds and if it is not set it defaults to 1 (filter disabled).

## TF tree
Our tf tree is composed of three transformation frames (world, odom and base_link):
- a static transformation links the world frame to the odom frame, the relation is defined in order to match the position of the data in the bags if necessary.
- the transformation between odom and base_link is the one responsible for the real localization of the robot in the space

## Custom message
- WheelSpeed: encodes the wheels rpm as specified in the assignment.

## How To Run

In this file you can also set the link between world and odom frame using static transformation.
The provided command will run all the nodes specified in the launch file and a precofigured rviz to look at the robot movement.

```console
$ roslaunch RoboticsProject main.launch
```
## Services

There are 2 services:
- `/reset_general` : take 3 parameter to set the position
- `/reset_zero` : reset the position to 0

these services can be used with:

```console
$ rosservice call /reset_general x y th
$ rosservice call /reset_zero
```

## Methodology
The main goal of the project is to compute the trajectory of the robot starting from the movement of the wheels.
In order to do that we compute the speed of each wheel in rpm to then compute the kinematic of the robot<sup>[1](#reference)</sup> and obtain the movement of the whole assembly. Having the speed of the robot we integrate using the 2 different method to obtain the position in each instant.
Comparing it with the true position recordered from the cameras we can see an error increasing with time. <br>We explained this error as such:<br>
This type of wheels generate a lot of forces on the joints and small flexions of the wheel axes can make the robot move in unexpected ways<sup>[2](#reference)</sup>, not accounted for by the model in use.
Moreover, the fact that the wheel can have small slippages for instance while performing a the direction change, can explain the the difference between the computed trajectory and the recorded one.<br>
The parameters were chosen by trial and error, modifying them and trying to reflect the bag data as closely as possible.<br>
This modification was done without the need to recompile the code at each test.


## Reference
- [Kinematic Model of a Four Mecanum Wheeled Mobile
Robot](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf)<sup>[1]</sup>
- [Motion Control and the Skidding of Mecanum-Wheel Vehicles](https://ijiset.com/vol5/v5s5/IJISET_V5_I05_10.pdf)<sup>[2]</sup>

## ToDo
- [x] create launch file with parameter
- [x] compute real wheels speed from wheels position
    - [x] custom message to pubblish the speed `WheelSpeed`  on topic `wheel_speed`
- [x] compute kinematic to obtain the speed of the robot
    - [x] publish speed on topic `cmd_vel` with a message of type `geometry_msgs/TwistStamped`
- [x] compute integration to obtain the position of the robot
    - [x] enumeration for the 2 integration methods
    - [x] add ROS parameter for initial position
    - [x] publish speed on topic `odom` with a message of type `nav_msgs/Odometry`
    - [x] Broadcast TF `odom->base_link`
- [x] compute inverse kineamtic from the speed of the robot to obtain the speed of the wheels
    - [x] custom message to publish the speed `WheelSpeed` on topic `wheels_rpm` 
- [x] tune parameter
- [x] create a service to reset the position to a given one
- [x] fix name in the diagrams
- [x] add reference material


