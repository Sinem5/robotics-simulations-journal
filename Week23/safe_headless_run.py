import os
import sys

# 1. HIJACK THE ENVIRONMENT BEFORE ISAAC BOOTS
# This forces Isaac to use CycloneDDS and ignores whatever python.sh tries to set
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

# 2. STRIP CONFLICTING SYSTEM PATHS
# We remove system ROS paths from Python's brain to stop the Segmentation Fault
sys.path = [p for p in sys.path if "/opt/ros" not in p]

# 3. NOW it is safe to start the simulator
from isaacsim import SimulationApp

config = {
    "headless": True,
    "width": 640,
    "height": 480,
    "renderer": "RayTracedLighting",
}
simulation_app = SimulationApp(config)

import carb
import omni.timeline
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim
import omni.isaac.core.utils.extensions as extensions

# [3] ENABLE EXTENSIONS WITH A SAFETY CHECK
print(">>> [1/4] ENABLING ROS 2 BRIDGE...")

# We enable the "meta" extension. Even though it warns "deprecated", 
# it handles the dependency loading better than calling the new one directly.
extensions.enable_extension("omni.isaac.ros2_bridge")
extensions.enable_extension("omni.isaac.sensor")

# FORCE A RENDER FRAME TO INITIALIZE EXTENSIONS
simulation_app.update()
simulation_app.update()

# Check if it actually loaded
import omni.ext
manager = omni.kit.app.get_app().get_extension_manager()
if not manager.is_extension_enabled("isaacsim.ros2.bridge"):
    carb.log_error("CRITICAL ERROR: ROS2 Bridge failed to load!")
    print(">>> STOP! The ROS Bridge crashed. Check your RMW configuration. <<<")
    # We continue anyway, but expect failure if this prints.
else:
    print(">>> SUCCESS: ROS 2 Bridge is ACTIVE and LOADED. <<<")

# [4] POTATO MODE
carb.settings.get_settings().set("/rtx/hydra/renderScale", 0.1)
carb.settings.get_settings().set("/rtx/reflections/enabled", False)

# [5] LOAD ROBOT
print(">>> [2/4] LOADING ROBOT...")
usd_path = "/home/sinem/recycler_ws/The_Recycler_Robot.usd"
omni.usd.get_context().open_stage(usd_path)

# Perform updates to let the USD load
for i in range(10):
    simulation_app.update()

world = World()
world.reset()

# [6] CHECK FOR BROKEN REFERENCES
# Your logs showed "Could not open asset". We check if the robot is actually there.
robot_prim_path = "/World/The_Recycler_Robot" 
my_robot = XFormPrim(prim_path=robot_prim_path)

if not my_robot.is_valid():
    print(f">>> WARNING: Robot not found at {robot_prim_path}")
    print(">>> Your USD might have broken paths. Checking '/Root' instead...")
    # Attempt to find it at the root if the previous path failed
    my_robot = XFormPrim(prim_path="/Root/The_Recycler_Robot")

# [7] START SIMULATION
print(">>> [3/4] STARTING TIMELINE...")
omni.timeline.get_timeline_interface().play()

# Warm up the graph
for i in range(60):
    world.step(render=True)

print(">>> [4/4] SIMULATION RUNNING. CHECK 'ros2 topic list' NOW! <<<")

# [8] RUN LOOP
frame = 0
try:
    while simulation_app.is_running():
        world.step(render=True)
        
        if frame % 200 == 0:
            if my_robot.is_valid():
                pos, _ = my_robot.get_world_pose()
                print(f"Frame {frame} | Robot Pos: {pos}")
            else:
                print(f"Frame {frame} | Robot Prim Invalid (USD Load Error)")
            
        frame += 1
        
except KeyboardInterrupt:
    simulation_app.close()