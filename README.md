# R2_bot

![IMG_2889](https://github.com/omo776/R2-bot/assets/88599328/a4de9d80-c98b-431e-81a7-600d6bdff23b)

A Food service robot built with the Turtlebot Burger model with an external USB camera, a LIDAR and two IR Sensors and two bags attached to its sides.


## Motivation
Service robots are increasingly being used in various industries to automate tasks and increase operational efficiency. For example, in the restaurant sector, these robots hold a lot of potential for transforming traditional, time-consuming service processes.
In this context, the emergence of advanced robotics platforms such as the Turtlebot, coupled with the development of software frameworks, namely ROS2 (Robot Operating System 2), has paved the way for the implementation of intelligent and autonomous service robots in restaurant settings. These platforms offer robust capabilities for perception, navigation, and manipulation, enabling robots to perform complex tasks in dynamic and unstructured environments.

## Tasks completed
To implement light detection and object detection for the purpose of decision making for the robot. Both of which require the use of the camera attached to turtlebot, to access the camera, the ROS2 Image Pipeline Tutorial (a series of git libraries/repositories) was used and tested in a linux environment. Once I was able to properly publish the image (frames) coming from R2D2’s camera to the topic “/image_raw”, I took the following steps

- **Collected, annotated a training and validation data set using Label Studio**
- **Trained both data sets using object_detector.ipynb and stored the .tflite file**
- **Created a ros subscriber to the “/image_raw” topic and collected each frame**
- **Used Mediapipe, Tensorflow, CVBridge and OpenCV to display the detected labels in the frames received**
- **Used OpenCV and HSV concept to identify specific leds lights in “ON” states in the frames received**


## Demo showing an operational turtlebot fulfiling orders, using Light and Food detection
View the demo here [Demo](https://drive.google.com/file/d/1Mp6J9l5eZZNH2oVENtqfElrkszVsy1E1/view)

View the light detection feature here [LED Light detction demo](https://drive.google.com/file/d/12tHoJDJxt9spCBk_9kUr4bh-te48sJua/view?resourcekey)



## Project objectives
The main objectives we aim to approve is to:
- **Implement the desired robot operations, including navigation, obstacle avoidance, and task execution**
- **Use ROS2 as the framework for software development to ensure reliability and scalability**
- **Integrate sensor components, computer vision algorithms, and machine learning techniques for perception and decision-making**
- **Test and demonstrate the feasibility and practicality of the proposed solution with real-world variables  and with ideal performance standards**


## In-Depth Technical report
View the report here [Technical report](https://docs.google.com/presentation/d/1eb9FhlfIevGrt4pKTMH5mopMgR1cYXbDe5jKMfd3KZs/edit#slide=id.p)
