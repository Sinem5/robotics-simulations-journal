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

class RealtimeSegmentNode(Node):
    def __init__(self):
        super().__init__('realtime_segment_node')
        try:
            self.model=YOLO('/home/sinem/new_ws/yolo11n-seg.engine', task='segment')
        except Exception as e:
            self.get_logger().error(f"MODEL FAILED TO LOAD: {e}")
            return
        
        self.get_logger().info("Model Loaded Succesfully")

        self.get_logger().info("Warming up model...")

        try:
            fake_image = np.zeros((720,1280,3), dtype=np.uint8)
            self.model(fake_image, verbose=False)
            self.get_logger().info("Model warmup complete.")
        except Exception as e:
            self.get_logger().error(f"MODEL WORMUP FAILED: {e}")
            return
        
        self.bridge = cv_bridge.CvBridge()

        self.image_queue = queue.Queue(maxsize=1)

        self.inference_thread = threading.Thread(target=self.inference_worker)
        self.inference_thread.daemon = True
        self.inference_thread.start()

        self.sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            qos_profile=self.sensor_qos_profile
        )
        self.result_publisher = self.create_publisher(
            Image,
            'segment_results',
            10
        )
        self.get_logger().info("Realtime Segmentor Node Started and Spinning")

    def image_callback(self, msg):
        try:
            self.image_queue.put_nowait(msg)
        except queue.Full:
            self.get_logger().warn("Skipping frame, inference thread is busy")

    def inference_worker(self):
        while rclpy.ok():
            try:
                msg=self.image_queue.get(timeout=1.0) #wait 1 sec
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                results = self.process_with_yolo(cv_image)
                processed_cv_image = self.process_segmentations(results, cv_image.copy())
                self.publish_results(processed_cv_image, msg)
                self.get_logger().info("Publish complete. Awaiting next image")
                self.image_queue.task_done()

            except queue.Empty:
                self.get_logger().debug("Queue was empty")
                pass
            except Exception as e:
                self.get_logger().error(f"Error in inference worker: {e}")

    def process_with_yolo(self, cv_image):
        results = self.model(cv_image, imgsz=640)
        return results
    
    def process_segmentations(self, results, original_image):
        # we will create a blank image with mask on it (color overlay) and blend with the original image 
        overlay = original_image.copy()
        if len(results) > 0:
            if results[0].masks is not None:
                for mask,box in zip(results[0].masks, results[0].boxes):
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    if confidence > 0.5:
                        points =np.array(mask.xy[0], dtype=np.int32)
                        cv2.fillPoly(overlay, [points], (0, 255, 0))
                        x1,y1,x2,y2 = map(int, box.xyxy[0])
                        label = f"Class {class_id}: {confidence:.2f}"
                        cv2.putText(original_image, label, (x1, y1-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        final_image = cv2.addWeighted(original_image, 0.6, overlay, 0.4, 0)
        return final_image

    def publish_results(self, processed_cv_image, original_msg):
        results_msg=self.bridge.cv2_to_imgmsg(processed_cv_image, 'bgr8')
        results_msg.header = original_msg.header
        self.result_publisher.publish(results_msg)


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    
    node = RealtimeSegmentNode()
    
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