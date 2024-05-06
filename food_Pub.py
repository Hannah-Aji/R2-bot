

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image


class ImageSubscriber(Node) :
    def __init__(self):
        super().__init__('imagesubscriber')

        self.image_subscriber = self.create_subscription(Image, "/image_raw", self.image_subscriber_callback, 10)

        self.base_options = python.BaseOptions(model_asset_path='/home/omo776/Documents/r2_bot/src/bot_vision/bot_vision/model(1).tflite')
        self.options = vision.ObjectDetectorOptions(base_options=self.base_options,
                                       score_threshold=0.75
                                    #    running_mode=vision.RunningMode.LIVE_STREAM
                                       )
        self.detector = vision.ObjectDetector.create_from_options(self.options)

    def image_subscriber_callback(self, image) :

        bridge = CvBridge()

        cv2image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2image)

        # rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        detection_result = self.detector.detect(mp_image)

        for i in range(len(detection_result.detections)): 
            self.get_logger().info("DETECTED OBJECT: {}".format(detection_result.detections[i]))


def main(args = None):

    rclpy.init(args = args)
    imageSub = ImageSubscriber()
    rclpy.spin(imageSub)
    imageSub.destroy_node()
    rclpy.shutdown()

     
if __name__ == '__main__':
    main()