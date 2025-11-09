from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import cv_bridge
import torch
from ultralytics import YOLO
import cv2
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.executors import MultiThreadedExecutor
import numpy as np

# New imports for the worker thread
import threading
import queue
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RealtimeDetectorNode(Node):
    def __init__(self):
        super().__init__('realtime_detector')
        self.get_logger().info("Node initialized.")

        # --- Load and Warm-up Model ---
        self.get_logger().info("Loading model now...")
        try:
            self.model = YOLO('/home/sinem/new_ws/yolo11n.engine', task='detect')
        except Exception as e:
            self.get_logger().error(f"MODEL FAILED TO LOAD: {e}")
            return
        self.get_logger().info("Model loaded successfully.")

        self.get_logger().info("Warming up model...")
        try:
            # Use a resolution that matches your camera
            fake_image = np.zeros((720, 1280, 3), dtype=np.uint8) 
            self.model(fake_image, verbose=False)
            self.get_logger().info("Model warmup complete.")
        except Exception as e:
            self.get_logger().error(f"MODEL WARMUP FAILED: {e}")
            return

        # --- End of Warm-up ---

        self.bridge = cv_bridge.CvBridge()

        # Create a thread-safe queue. maxsize=1 is our frame-skipping.
        self.image_queue = queue.Queue(maxsize=1)

        # Create and start the dedicated inference thread
        self.inference_thread = threading.Thread(target=self.inference_worker)
        self.inference_thread.daemon = True  # Allows script to exit with Ctrl+C
        self.inference_thread.start()

        # Define a QoS profile that matches sensor data
        self.sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback, # This is now the "producer"
            qos_profile=self.sensor_qos_profile
        )
        self.result_publisher = self.create_publisher(
            Image,
            'detection_results',
            10
        )
        self.get_logger().info("Realtime Detector Node Started and Spinning.")


    def image_callback(self, msg):
        try:
            self.image_queue.put_nowait(msg)
        except queue.Full:
            # We use .warn() for skipped frames, .info() for normal flow
            self.get_logger().warn("Skipping frame, inference thread is busy.")


    def inference_worker(self):
        """
        This is the "Consumer" function with verbose logging.
        """
        self.get_logger().info("Inference worker thread started.")
        while rclpy.ok():
            try:
                # --- NEW LOG 1 ---
                self.get_logger().info("Waiting for new image in queue...")
                msg = self.image_queue.get(timeout=1.0) # Wait 1 sec
                
                # --- NEW LOG 2 ---
                self.get_logger().info("Got image! Converting to CV2...")
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                
                # --- NEW LOG 3 ---
                self.get_logger().info(f"Image converted ({cv_image.shape}). Starting YOLO inference...")
                
                # 1. Process with YOLO (This is where it will freeze)
                results = self.process_with_yolo(cv_image)
                
                # --- NEW LOG 4 ---
                self.get_logger().info("Inference complete. Drawing boxes...")
                
                # 2. Draw detections
                processed_cv_image = self.process_detections(results, cv_image.copy())
                
                # --- NEW LOG 5 ---
                self.get_logger().info("Boxes drawn. Publishing results...")

                # 3. Publish results
                self.publish_results(processed_cv_image, msg)
                
                # --- NEW LOG 6 ---
                self.get_logger().info("Publish complete. Awaiting next image.")

                self.image_queue.task_done()

            except queue.Empty:
                # This is normal, just means no new images in 1 second
                # We log this at a 'debug' level so it doesn't spam
                self.get_logger().debug("Queue was empty.")
                pass
            except Exception as e:
                self.get_logger().error(f"Error in inference worker: {e}")


    def process_with_yolo(self, cv_image):
        results = self.model(cv_image, imgsz=640)
        return results

    def process_detections(self, results, original_image):
        # (This function is unchanged)
        if len(results) > 0:
            boxes = results[0].boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    if confidence > 0.5:
                        cv2.rectangle(original_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"Class {class_id}: {confidence:.2f}"
                        cv2.putText(original_image, label, (x1, y1 - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return original_image

    def publish_results(self, processed_cv_image, original_msg):
        # (This function is unchanged)
        result_msg = self.bridge.cv2_to_imgmsg(processed_cv_image, 'bgr8')
        result_msg.header = original_msg.header
        self.result_publisher.publish(result_msg)


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    
    node = RealtimeDetectorNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down node...")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()