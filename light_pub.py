

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge



class GreenLightDetect(Node):

    def __init__(self):
        super().__init__('light_detect')

        self.camera_subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )

        self.publisher_light = self.create_publisher(String, '/light_status', 10)

    
    
    def image_callback(self, image):
        msg = String()

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        # Convert the frame to HSV format
        hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Convert the frame to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply a threshold to create a binary image
        _, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

        # Find contours in the binary image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on area and shape
        min_area = 10  # Adjust this threshold according to your needs
        leds = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:
                leds.append(contour)

        # Iterate over detected LEDs
        for contour in leds:
            x, y, w, h = cv2.boundingRect(contour)
            
            hsv_color = hsv_frame[y:y+h, x:x+w].mean(axis=(0, 1))  # Calculate average HSV color
            
            # Round HSV values to the nearest whole number
            hsv_color = np.round(hsv_color).astype(int)

            #print(f"hsv: {hsv_color}")

            # Check if the HSV color falls within the specified range for green light
            if (45 <= hsv_color[0] <= 70 and 210 <= hsv_color[2] <= 255):
                #print("green light")230 <= hsv_color[2] <= 255
                
                msg.data = "green light"
                self.publisher_light.publish(msg)
                self.get_logger().info("Publishing detected: {}".format(msg.data))


    
        # cv2.waitKey(1)  # This line is necessary to update the OpenCV window
        cv2.destroyAllWindows()
    



def main(args=None):
    rclpy.init(args=args)
    light_detect_node= GreenLightDetect()
    rclpy.spin(light_detect_node)
    light_detect_node.destroy_node()
    rclpy.shutdown()

     
if __name__ == '__main__':
    main()