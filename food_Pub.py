



import rclpy
from rclpy.node import Node
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from cv_bridge import CvBridge
from std_msgs.msg import String

from sensor_msgs.msg import Image


class FoodPub(Node):

    def __init__(self):
        super().__init__('food_pub')

        self.image_subscriber = self.create_subscription(
            Image,
            "/image_raw_laptop",
            self.image_subscriber_callback,
            10
        )
  
        self.food_publisher = self.create_publisher(String, '/Food_Detection', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.food_callback)

        self.base_options = python.BaseOptions(model_asset_path='/home/omo776/Documents/r2_bot/src/bot_vision/bot_vision/model(1).tflite')
        self.options = vision.ObjectDetectorOptions(base_options=self.base_options,
                                       score_threshold=0.75
                                    #    running_mode=vision.RunningMode.LIVE_STREAM
                                       )
        self.detector = vision.ObjectDetector.create_from_options(self.options)
        self.detection_result = None



    def image_subscriber_callback(self, image):

        bridge = CvBridge()
        cv2image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2image)
        
        # rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.detection_result = self.detector.detect(mp_image)
        if self.detection_result is not None:
            msg = String()
            for i in range(len(self.detection_result.detections)): 
                msg.data = self.detection_result.detections[i].categories[0].category_name
                self.food_publisher.publish(msg)
                self.get_logger().info("Publishing detected: {}".format(msg.data))

                                





def main(args=None):
    rclpy.init(args=args)
    food_pub = FoodPub()
    rclpy.spin(food_pub)
    food_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
