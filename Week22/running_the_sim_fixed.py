from isaacsim import SimulationApp
import carb


# [1] CONFIGURE FOR MAXIMUM PERFORMANCE
# headless=True is the key to saving your GPU. No window will open.
config = {
    "headless": True, 
    "width": 640,  # Minimal resolution for sensors
    "height": 480,
    "renderer": "RayTracedLighting", # Lowest quality renderer
}

print(">>> INITIALIZING HEADLESS SIMULATION (Please wait)...")
simulation_app = SimulationApp(config)

# [2] ENABLE ROS 2 BRIDGE MANUALLY
import omni.isaac.core.utils.extensions as extensions
print(">>> ENABLING ROS 2 BRIDGE...")
extensions.enable_extension("omni.isaac.ros2_bridge")
extensions.enable_extension("omni.isaac.sensor")

# [3] AGGRESSIVE GRAPHICS REDUCTION (The "Potato" Settings)
# We set these to practically zero to ensure your 4GB VRAM survives
print(">>> OPTIMIZING GRAPHICS FOR RTX 3050...")
settings = carb.settings.get_settings()
settings.set("/rtx/hydra/renderScale", 0.1) # Render at 10% resolution internally
settings.set("/rtx/reflections/enabled", False)
settings.set("/rtx/translucency/enabled", False)
settings.set("/rtx/shadows/enabled", False)
settings.set("/rtx/indirectDiffuse/enabled", False)
settings.set("/rtx/directLighting/sampledLighting/enabled", False)

import omni.timeline
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim

# [4] LOAD THE ROBOT
print(">>> LOADING ROBOT STAGE...")
usd_path = "/home/sinem/recycler_ws/The_Recycler_Robot.usd"
omni.usd.get_context().open_stage(usd_path)

print(">>> CREATING WORLD...")
world = World()
# We perform one update to ensure the stage is loaded
simulation_app.update()

# [5] FIND THE ROBOT
# Make sure this path is correct for your USD!
robot_prim_path = "/World/The_Recycler_Robot"
my_robot = XFormPrim(prim_path=robot_prim_path)

# [6] START SIMULATION
world.reset()
print(">>> FORCING TIMELINE TO PLAY... <<<")
omni.timeline.get_timeline_interface().play()
print(">>> SIMULATION STARTED IN BACKGROUND! <<<")
print(">>> CHECK YOUR OTHER TERMINAL FOR 'ros2 topic list' <<<")

# [7] RUNNING LOOP
frame_count = 0
try:
    while simulation_app.is_running():
        # We MUST set render=True for cameras/sensors to work, 
        # even in headless mode. But our renderScale is 0.1, so it's cheap.
        world.step(render=True)
        
        if frame_count % 100 == 0:
            if my_robot.is_valid():
                pos, _ = my_robot.get_world_pose()
                print(f"Status: Running | Frame: {frame_count} | Robot Pos: {pos}")
            else:
                print(f"Status: Running | Frame: {frame_count} | Robot not found yet...")
        
        frame_count += 1
        
except KeyboardInterrupt:
    print(">>> STOPPING SIMULATION... <<<")
    simulation_app.close()