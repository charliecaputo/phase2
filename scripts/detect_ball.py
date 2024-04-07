#!/usr/bin/env python3
# import important stuff
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Initalize node
rospy.init_node('detect_ball')
# Set up as publisher
image_pub = rospy.Publisher('/ball_2D', Image, queue_size=10)
bridge = CvBridge()

# Bridge ROS to OpenCV
def image_callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
    processed_image = detect_ball(cv_image)
    publish_image(processed_image)

def detect_ball(cv_image):
    # Convert RGB image to HSV Using notation online
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

    # Define range of ball color in HSV
    lower_blue = np.array([10,100,1])##20,70,1
    upper_blue = np.array([60,255,255])

    # Threshold the HSV image to get only the balls color
    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

    # Remove background noise
    # This is a tight fit but works for the scenario
    roi = np.zeros(cv_image.shape[:2], dtype=np.uint8)
    cv2.rectangle(roi, (350, 200), (cv_image.shape[1] - 350, cv_image.shape[0] - 200), 255, -1)
    
    # Bitwise and function
    masked_image = cv2.bitwise_and(mask, mask, mask=roi)
    
    # Return the desired image (Hopefully)
    return masked_image

# Send image to ros after processing
def publish_image(cv_image):
    ros_image = bridge.cv2_to_imgmsg(cv_image, "mono8")
    image_pub.publish(ros_image)

if __name__ == '__main__':
    # Get image from /image_raw
    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()   
