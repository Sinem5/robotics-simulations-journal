from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import cv_bridge
import torch
from ultralytics import YOLO
import cv2
from visualization_msgs.msg import Marker, MarkerArray

class RealtimeSegmentationDetector(Node):
    def __init__(self):
        super().__init__('realtime_segmentation_detector')
        self.bridge = cv_bridge.CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            10
        )
        # self.model = torch.load('/home/sinem/ros2_ws/yolo11n.pt') it's not for yolo models
        self.model = YOLO('/home/sinem/ros2_ws/yolo11n.pt')

        self.result_publisher = self.create_publisher(
            Image,
            'detection_results',
            10
        )

    def image_callback(self, msg):
        try:
            # convert ros2 image -> cv2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height,width, channels = cv_image.shape
            print(f"Image size: {width}x{height}")

            # process the image
            results = self.process_with_yolo(cv_image)
            processed_cv_image = self.process_detections(results, cv_image.copy())
            self.publish_results(processed_cv_image, msg)

        except Exception as e:
            self.get_logger().error(f"Conversion failed: {e}")
    
    def process_with_yolo(self, cv_image):
        results = self.model(cv_image, imgsz=640)
        return results
    
    def process_detections(self, results, original_image):
        if len(results) > 0:
            boxes = results[0].boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])

                    # only draw boxes for confidence > 0.5
                    if confidence > 0.5:
                        # Draw bounding box
                        cv2.rectangle(original_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"Class {class_id}: {confidence:.2f}"
                        cv2.putText(original_image, label, (x1, y1 - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return original_image
    
    def publish_results(self, processed_cv_image, original_msg):
        # Convert cv2 image -> ros2 image
        result_msg = self.bridge.cv2_to_imgmsg(processed_cv_image, 'bgr8')
        result_msg.header = original_msg.header  # Keep original header info
        self.result_publisher.publish(result_msg)

    def create_bbox_marker(self, x1, y1, x2, y2, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue
        return marker
    
def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)

    node = RealtimeSegmentationDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
