from sensor_msgs.msg import Image
import cv_bridge

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

    def image_callback(self, msg):
        try:
            # convert ros2 image -> cv2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height,width, channels = cv_image.shape
            print(f"Image size: {width}x{height}")
        except Exception as e:
            self.get_logger().error(f"Conversion failed: {e}")

