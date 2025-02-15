import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Create a publisher
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)

        # Create a timer to publish images periodically
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a CvBridge object to convert OpenCV images to ROS messages
        self.bridge = CvBridge()

        # Open the webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Error: Could not open camera.')

    def timer_callback(self):
        # Capture frame from webcam
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error('Error: Could not read frame.')
            return

        # Convert the OpenCV image (numpy array) to a ROS Image message
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publish the image message
        self.publisher_.publish(image_message)

        self.get_logger().info('Publishing image')

    def destroy(self):
        # Release the camera when shutting down
        self.cap.release()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
