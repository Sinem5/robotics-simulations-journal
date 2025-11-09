## Week 14 Isaac Sim Script (Fixed)

https://github.com/user-attachments/assets/064e93a0-47a2-4442-85d8-b05fc0c37553

I encountered an issue where Isaac Sim froze after I ran the script.

I added a self.is_processing flag to prevent the script from trying to process all images simultaneously, which should help free GPU memory and avoid blocking Isaac Sim’s GPU usage. However, this approach didn’t work as expected.

I also tried using a MultiThreadExecutor to prevent the spin thread from being blocked, but that didn’t resolve the issue either.

As the next step, we will use a .engine model instead of a .pt model, since .engine files can share GPU resources more efficiently.
To achieve this, we’ll create a separate [script to export the model](export_model.py) to .engine format using Ultralytics.


To do this, run the export_model.py script. Note that the export process may take some time and will utilize your GPU.
Make sure to activate your virtual environment before running the script.

After the export is complete, open realtime_seg_det.py and replace the .pt model path with the new .engine file.
Then, try running the script again. Unfortunately, Isaac Sim froze once more.

Again, it didn’t work. There was a small warning about the model task, so I modified the script to perform only detection for now. The goal is to get it working first, and we can add the other parts later.

Apparently, we need to warm up CUDA and TensorRT because the first image inference consumes a lot of resources, which causes a conflict in GPU usage. To handle this, we send an empty image to warm up the model before real inference begins. However, after the warm-up, Isaac Sim froze again.

I also tried running Isaac Sim in headless mode, but it still didn’t seem to help.

Additionally, I forgot to mention that I added an inference worker section to the script. I’ll share the full script in this week’s file system: [The Script](detect_rt.py)

Actually, after some adjustments, I managed to get it running, so a few of the previous details are no longer relevant

After setting up, I ran Isaac Sim first, then executed the following commands in new terminal:

```
cd [workspace directory]
source yolov8_venv/bin/activate
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python [path to the script]
```

Initially, I encountered a package conflict with NumPy.
To fix this, run:

```
pip install "numpy<2.0"
```

After this fix, everything worked!

To visualize the results in RViz, go to the Display window and set the Image topic to:

```
/detection_results
```

For reference, here’s the full list of available ROS 2 topics:
```
$ ros2 topic list
/clicked_point
/detection_results
/goal_pose
/initialpose
/parameter_events
/rgb
/rosout
/tf
/tf_static

```


I’ll continue working on extending this for segmentation as well. It’s also possible to train using synthetic data.
I’ll be taking a one-week break due to my midterms.