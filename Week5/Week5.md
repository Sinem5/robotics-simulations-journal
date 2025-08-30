## Week 5 Scripting Real-Time Detection and Segmentation with YOLO in Isaac Sim (Failed), Webots First Steps

### The Failure in Isaac Sİm

 Over the past two weeks, I’ve been trying to get a script running in IsaacSim, but I faced a lot of Python-related problems. At first, I tried to solve it with [pyenv](https://github.com/pyenv/pyenv?tab=readme-ov-file), which helped a little, but I still kept running into issues with certain packages. Basically, I was using ROS 2 Foxy, which depends on Python 3.8, while IsaacSim is built for Python 3.10. That version mismatch caused many of the problems I was seeing. If anyone is using ROS 2 Humble, you might have better luck running this script, but I can’t guarantee it since I didn’t manage to fully test it myself.
 [The Script](Week5/realtime_seg_det.py)

For anyone who wants to explore this more, I’ll share three sources: </br>
 [one about how to script in IsaacSim](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/environment_setup.html),</br> 
[another about building YOLO projects with scripting inside IsaacSim](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection/blob/main/isaac_ros_yolov8/scripts/isaac_ros_yolov8_visualizer.py),</br>
 [and a third that covers YOLO in IsaacSim but not directly with scripting.](https://www.youtube.com/watch?v=NnKNfmJimMU) 
 
 I only wrote my script with a little help from AI guidance, so it’s not fully tested yet, but I still want to share it in case it helps others who are interested in this area. For now, I’ve decided to pause on IsaacSim because of all these compatibility problems and continue my weekly projects in Webots, where things are more stable and I can keep making progress.


### Webots First Steps

https://github.com/user-attachments/assets/ae36d3e5-439e-4734-b1e7-a2e8df04c6ee

I explored alternatives to IsaacSim and found that Webots is highly regarded among users. Many praise its extensive features and ease of use, which encouraged me to learn it. Since I’m starting from scratch with Webots and robotics in general, I anticipate some challenges.

For installation, I recommend visiting [the official Webots website](https://cyberbotics.com/)
. If you’re using Ubuntu, you can download the Debian package or install it from the Snap Store. Additionally, there are [official documents](https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots) available for various tutorials. While many YouTube tutorials exist, most are based on older versions of Webots. I’m using the latest version, so there may be slight differences, but the essential processes remain the same.

Recently, I followed [a tutorial](https://www.youtube.com/watch?v=luyg3plGujg&list=PLbEU0vp_OQkUwANRMUOM00SXybYQ4TXNF)
 on how to create a world in Webots. I managed to add one cube and integrate physics into it. Unlike IsaacSim, where physics are already integrated, in Webots, you need to configure it yourself, as shown in the tutorial video.

I plan to continue learning Webots over the next few weeks and hope that scripting will become easier. However, I'm still uncertain about that. See you next week!











