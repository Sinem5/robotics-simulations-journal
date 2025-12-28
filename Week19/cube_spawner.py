import omni
import numpy as np
from omni.isaac.core.objects import DynamicCuboid
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import torch
import queue
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class CubeSpawner(Node):

    def __init__(self):

        super().__init__('cube_spawner_node')
        
        self.bridge = cv_bridge.CvBridge()

        self.image_queue = queue.Queue(maxsize=1)

        self.desk_bounds = {
            'x_min':-0.76446,
            'x_max':0.72602,
            'y_min':-0.68078,
            'y_max':0.72925,
            'z_height':0.07565
        }

        self.main_timer = self.create_timer(0.5, self.logic_loop)

        self.sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.camera_sub = self.create_subscription(
            Image,
            'desk_camera_topic',
            self.image_callback,
            qos_profile=self.sensor_qos_profile
        )

        stage = omni.usd.get_context().get_stage()

    def spawn_cube(self, color_name):
        rand_x = np.random.uniform(self.desk_bounds['x_min'],self.desk_bounds['x_max'])
        rand_y = np.random.uniform(self.desk_bounds['y_min'],self.desk_bounds['y_max'])

        unique_suffix = str(int(self.get_clock().now().nanoseconds))
        prim_path = f"/Root/blocks/{color_name}_{unique_suffix}"

        if color_name == "red":
            rgb = np.array([1.0,0.0,0.0])
        else:
            rgb = np.array([0.0,1.0,0.0])

        DynamicCuboid(
            prim_path=prim_path,
            name=f"cube_{unique_suffix}",
            position=np.array([rand_x,rand_y, self.desk_bounds['z_height']]),
            scale=np.array([0.15,0.15,0.15]),
            color=rgb
        )
        self.get_logger().info(f"Spawned {color_name} cube at {prim_path}")

    def logic_loop(self):
        if self.image_queue.empty():
            return
        
        ros_image = self.image_queue.get()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Failed: {e}")
            return
        
        has_red = self.check_color_presence(cv_image, "red")
        has_green = self.check_color_presence(cv_image, "green")

        if not has_red: 
            self.spawn_cube("red")
        
        if not has_green:
            self.spawn_cube("green")

    def check_color_presence(self, image, color_name):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        if color_name == "red":
            lower = np.array([0, 120, 70])
            upper = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower, upper)
            mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
            mask = mask1 + mask2
        elif color_name == "green":
            lower = np.array([40, 40, 40])
            upper = np.array([80, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)

        if cv2.countNonZero(mask) > 500:
            return True
        return False


    def image_callback(self, msg):
        try:
            self.image_queue.put_nowait(msg)
        except queue.Full:
            # We use .warn() for skipped frames, .info() for normal flow
            self.get_logger().warn("Skipping frame, inference thread is busy.")

if not rclpy.ok():
    rclpy.init()

# Global variable to persist in Script Editor
global spawner_node
spawner_node = CubeSpawner()

def on_physics_step(step_size):
    if rclpy.ok():
        rclpy.spin_once(spawner_node, timeout_sec=0.0)

import omni.physx
try:
    sub_handle = None 
    physx_interface = omni.physx.get_physx_interface()
    # Subscribe to physics steps
    sub_handle = physx_interface.subscribe_physics_step_events(on_physics_step)
    print("✅ Spawner Node Started! Press 'Play' to begin.")
except Exception as e:
    print(f"Error attaching to physics: {e}")