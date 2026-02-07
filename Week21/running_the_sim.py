from isaacsim import SimulationApp
import carb

carb.log_warn(">>> [1/5] STARTING SIMULATION APP... (This takes 10s)")
config = {"headless": False, 
    "width": 800, 
    "height": 600, 
    "renderer": "RayTracedLighting"}
simulation_app = SimulationApp(config)

import omni.usd
from omni.isaac.core import World
import omni.kit.commands

carb.log_warn(">>> [3/5] LOADING ROBOT... (THIS IS WHERE IT FREAZES - WAIT!)")
usd_path = "/home/sinem/recycler_ws/The_Recycler_Robot.usd"
omni.usd.get_context().open_stage(usd_path)

def set_potato_mode():
    carb.log_warn(">>> [2/5] APP STARTED. ACTIVATING POTATO MODE...")
    # Force low resolution scaling (Blurry but fast)
    carb.settings.get_settings().set("/rtx/hydra/renderScale", 0.5)
    # Limit texture memory to 500MB
    carb.settings.get_settings().set("/rtx/hydra/texture/memoryBudget", 500)
    # Turn off fancy reflections
    carb.settings.get_settings().set("/rtx/reflections/enabled", False)
    carb.settings.get_settings().set("/rtx/indirectDiffuse/enabled", False)
    print(">>> POTATO MODE ACTIVATED: Graphics lowered to save GPU <<<")

set_potato_mode()

carb.log_warn(">>> [4/5] ROBOT LOADED! CREATING WORLD...")
world = World()
world.reset()

print("Simulation started. Look at the window!", flush=True)

import time

carb.log_warn(">>> [5/5] SUCCESS! SIMULATION LOOP STARTING <<<")
print(">>> APP IS READY. WAITING FOR GRAPHICS... <<<", flush=True)

try:
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.current_time_step_index % 1000 == 0:
            print(f"I AM ALIVE! Frame: {world.current_time_step_index} - Robot Position: {world.scene.get_object('Nova_Carter').get_world_pose()[0]}", flush=True)
   
except KeyboardInterrupt:
    carb.log_warn(">>> STOPPING SIMULATION... <<<")    
    simulation_app.close()