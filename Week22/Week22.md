## Week22 Problems After Problems
Nova Carter already has an Action Graph if you are using the Nova Carter ROS version. I am trying to get my simulation working, but I am facing several issues.

First, my robots were not linked correctly. I am currently trying to fix this by linking them again, but I am experiencing many crash reports in Isaac Sim. Most of these crashes seem to be caused by the 3D LiDAR. I deleted the 3D LiDAR and also removed the related nodes from the Action Graph, but the crashes are still happening.

I am not completely sure yet how to correctly link my robots, but I am working on it. The steps I followed one week ago should be correct. I believe the connection itself may not be entirely wrong; it could be that my computer does not have enough resources to process everything, which leads to the crashes. If the issue is not hardware-related, I may need to change the robot configuration or try a different setup. I am not certain yet, I will evaluate this during the process.

I also detected a possible issue with the running script. To debug this, I will temporarily reconnect the robot using the previous (incorrect) method so I can test whether the problem comes from the script or from a corrupted connection. However, it appears that the problem is not related to the script.

When I run the simulation using the script, I do not see any Action Graphs listed in the simulation. This suggests that there is a problem with how the Action Graphs are being executed. Since my GPU is not powerful enough, I need to use headless mode. You can find the script here:
[running_the_sim_fixed.py](running_the_sim_fixed.py)

Make sure you do not source ROS in the same terminal where you run this script.

Apparently, the problem is not with the script, something is wrong with the Action Graphs. I do not see any topics being listed. I planned to try the following steps:

- Find the "On Playback Tick" node in your Action Graph.

- Delete it.

- Replace it with an "On Tick" node. (This triggers on every simulation step, regardless of the "Play/Pause" state).

- Save and try your headless script again.

However, I cannot try these steps because the "On Playback Tick" node is an ancestor node, which means Isaac Sim does not allow me to modify it.

At this point, I feel a little lost, so let me clarify my goal. I need to generate a map of the room, which means I need SLAM. For SLAM to work, I need the appropriate ROS topics. The robot also needs to move around to perform mapping.

It seems that the only option left is to create the Action Graph myself so that I can use teleop_twist_keyboard with SLAM. Writing this down actually helped clarify my thoughts, at least I now have a direction to try.

Additionally, I may need to switch to another robot model. At the moment, my current robot connection appears to be severed, and I have not been able to find a way to fix or correctly reconfigure it. I still believe the steps I followed are correct, but my computer may not be powerful enough to handle the full setup. Because of this, I might need to use a different robot while continuing this project. However, I am not completely certain, time will show whether this is necessary.

I will try to finish the Action Graph as soon as possible so that I can demonstrate the SLAM part quickly. Hopefully, I can share the results next week. See you then!