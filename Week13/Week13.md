## Week 13 One Step Closer to Getting Isaac Sim Running Smoothly

This week, I ran into several driver and system configuration issues while setting up my environment. Fixing them took some time.

 I have included a link to NVIDIA’s robotics courses:
</br>
https://www.nvidia.com/en-us/learn/learning-path/robotics/?ncid=ref-inor-523702-vt48

The GPU issue has been resolved. Isaac Sim was not utilizing the GPU initially, but reinstalling the driver fixed the problem.

I have reinstalled ROS 2.

I encountered a cv_bridge module not found error. After following the solution from the forum link below, running the suggested command, and rebuilding my workspace, the issue was resolved.

https://forums.developer.nvidia.com/t/ros2-bridge-not-loading/276129/4

```
sudo apt install ros-humble-cv-bridge
```

I ran into an ultralytics module not found error, too. The problem was fixed after creating a new virtual environment and installing ultralytics with the commands listed below.

```
python3 -m venv yolov8_venv
source yolov8_venv/bin/activate
pip install ultralytics
```

Steps to get the terminal ready:

```
cd ~/{isaacsimfolder}
source ~/yolov8_venv/bin/activate
source /opt/ros/{ros-distro}/setup.bash
./isaac-sim.sh
```

After opening the USD file, I ran the script from the Script Editor. There were no errors, but Isaac Sim froze and stopped using the GPU. I’ll look into the issue and check the results next week.