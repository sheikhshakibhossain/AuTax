import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ImageSubscriberNode(Node):
    def __init__(self):
        super().__init__('image_subscriber_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/webcam_image/compressed',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        # Decompress the image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display the image
        cv2.imshow("Received Image", image)
        cv2.waitKey(1)  # Wait for a short time to update the window

def main(args=None):
    rclpy.init(args=args)
    image_subscriber_node = ImageSubscriberNode()
    
    try:
        rclpy.spin(image_subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        image_subscriber_node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()