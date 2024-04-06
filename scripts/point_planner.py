#!/usr/bin/env python3

import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist, PointStamped
from ur5e_control.msg import Plan
import tf2_geometry_msgs
from robot_vision_lectures.msg import SphereParams

def sphere_callback(data):
    global ball_position
    ball_position = PointStamped()
    ball_position.header.stamp = rospy.Time()
    ball_position.header.frame_id = "camera_color_optical_frame"  # Assuming the ball position is in the camera frame
    ball_position.point.x = data.xc
    ball_position.point.y = data.yc
    ball_position.point.z = data.zc
    rospy.loginfo("Ball position: x=%f, y=%f, z=%f", ball_position.point.x, ball_position.point.y, ball_position.point.z)

if __name__ == '__main__':
    rospy.init_node('simple_planner', anonymous=True)
    plan_pub = rospy.Publisher('/plan', Plan, queue_size=10)
    loop_rate = rospy.Rate(10)
    plan = Plan()

    # Initialize tf2 listener and broadcaster
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Subscriber for receiving sphere parameters
    sphere_sub = rospy.Subscriber('/sphere_params', SphereParams, sphere_callback)

    while not rospy.is_shutdown():
        try:
            # Get the transform from the camera frame to the base frame
            trans = tf_buffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())

            # Transform the ball position from camera frame to base frame
            ball_position_base = tf2_geometry_msgs.do_transform_point(ball_position, trans)

            # Point 1
            plan_point1 = Twist()
            plan_point1.linear.x = -0.4
            plan_point1.linear.y = -0.23
            plan_point1.linear.z = 0.5
            plan_point1.angular.x = 3.14
            plan_point1.angular.y = 0.0
            plan_point1.angular.z = 1.57
            plan.points.append(plan_point1)

            # Add the transformed ball position as point 2
            ball_twist = Twist()
            ball_twist.linear = ball_position_base.point
            ball_twist.angular.x = 3.14
            ball_twist.angular.y = 0.0
            ball_twist.angular.z = 1.57
            plan.points.append(ball_twist)
            
            # Point 3
            plan_point3 = Twist()
            plan_point3.linear.x = -0.7
            plan_point3.linear.y = -0.23
            plan_point3.linear.z = 0.5
            plan_point3.angular.x = 3.14
            plan_point3.angular.y = 0.0
            plan_point3.angular.z = 1.57
            plan.points.append(plan_point3)

            # Point 4
            plan_point4 = Twist()
            plan_point4.linear.x = -0.7
            plan_point4.linear.y = -0.23
            plan_point4.linear.z = 0.1
            plan_point4.angular.x = 3.14
            plan_point4.angular.y = 0.0
            plan_point4.angular.z = 1.57
            plan.points.append(plan_point4)

            # Publish the plan
            plan_pub.publish(plan)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to get transform from 'camera_color_optical_frame' to 'base'")

        loop_rate.sleep()
