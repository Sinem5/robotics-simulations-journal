## Week24 Driving the Robot 

Above the root joint, there were components called 'world' and 'world2base'. I deleted both because they were leftover connections. However, I still have the following error:
 
```
2026-02-20 12:10:51 [17,843ms] [Error] [omni.physx.plugin] RigidBody (/Root/rs007n_onrobot_rg2/base_link) appears to be a part of a closed articulation, which is not supported, please exclude one of the joints:
/Root/Nova_Carter_ROS/joint_caster_base
/Root/Nova_Carter_ROS/joint_swing_left
/Root/Nova_Carter_ROS/joint_swing_right
/Root/Nova_Carter_ROS/joint_caster_left
/Root/Nova_Carter_ROS/joint_caster_right
/Root/rs007n_onrobot_rg2/root_joint
/Root/Nova_Carter_ROS/dummy_link/FixedJoint
/Root/Nova_Carter_ROS/dummy_link_2/FixedJoint
/Root/Nova_Carter_ROS/joint_wheel_left
/Root/Nova_Carter_ROS/joint_wheel_right
/Root/rs007n_onrobot_rg2/base_link/joint1
/Root/rs007n_onrobot_rg2/link1/joint2
/Root/rs007n_onrobot_rg2/link2
 from articulation, the joint will be now excluded from the articulation.
```

Eureka!!!

I discovered that there was another joint called 'AssemblerFixedJoint.' Once I deleted it, the error disappeared.

Now, it's time to try using teleop_twist_keyboard. 

Run the simulation from the script as we did last week. Check the topics, and in a separate terminal, run the following command:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
and in another terminal run 

```
rviz2
```

In RViz, I opened one of the cameras to see if it's driving. I chose the left camera, but you can open any of them, or all of them if you prefer.

https://github.com/user-attachments/assets/868f05ed-bdcf-4cb2-b493-d47c836f0166


These are the keyboard controls:

 l -> turn right </br>
 j -> turn left </br>
 i -> forward </br>
 , -> backward </br>
 k -> break </br>


I realize that our cubes don't have the correct physics and are floating. I will address this issue later when it's time to pick them up.


I checked some topic information to get on next step which is SLAM. I also add my outputs so you can see.

[A cheatsheet for ROS commands](https://www.theconstruct.ai/wp-content/uploads/2021/10/ROS2-Command-Cheat-Sheets-updated.pdf)



```
ros2 topic info /cmd_vel 

Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1


ros2 topic info /scan

Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscription count: 0


ros2 topic info /chassis/odom 

Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 0

ros2 topic info /tf

Type: tf2_msgs/msg/TFMessage
Publisher count: 4
Subscription count: 0
```

```
 ros2 topic hz /scan 

average rate: 5.734
	min: 0.169s max: 0.179s std dev: 0.00361s window: 7
average rate: 5.710
	min: 0.169s max: 0.181s std dev: 0.00380s window: 13
average rate: 5.694
	min: 0.169s max: 0.181s std dev: 0.00377s window: 19

ros2 topic hz /chassis/odom 

average rate: 11.312
	min: 0.001s max: 0.185s std dev: 0.08666s window: 12
average rate: 11.392
	min: 0.001s max: 0.185s std dev: 0.08591s window: 24
average rate: 11.412
	min: 0.001s max: 0.185s std dev: 0.08585s window: 36

ros2 topic hz /tf 

average rate: 45.492
	min: 0.000s max: 0.178s std dev: 0.05599s window: 48
average rate: 45.846
	min: 0.000s max: 0.178s std dev: 0.05516s window: 96
average rate: 45.936
	min: 0.000s max: 0.178s std dev: 0.05510s window: 144

```

Now it is time for SLAM.

To download the SLAM (for humble):

```
sudo apt install ros-humble-slam-toolbox
```

To run the SLAM:

```
ros2 launch slam_toolbox online_sync_launch.py 
```

I realize our scan topic is not publishing the data we want according to the following command: 

```
ros2 topic echo /scan
```

This command returns a series of 0s and 1s, indicating that my LiDAR has some sort of issue. 

I will investigate this and run SLAM next week, I hope.
