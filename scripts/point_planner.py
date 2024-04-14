#!/usr/bin/env python3

import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist, PointStamped
from ur5e_control.msg import Plan
import tf2_geometry_msgs
from robot_vision_lectures.msg import SphereParams

def calculate_distance():
    try:
        # Get the transform from the camera frame to the base frame
        trans = tf_buffer.lookup_transform("camera_color_optical_frame", "base", rospy.Time())
        
        # Extract the translation components (x, y, z)
        distance_x = trans.transform.translation.x
        distance_y = trans.transform.translation.y
        distance_z = trans.transform.translation.z
        
        return distance_x, distance_y, distance_z

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to get transform from 'camera_color_optical_frame' to 'base'")
        return None, None, None

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
    rospy.init_node('planner_with_distance_and_ball', anonymous=True)
    plan_pub = rospy.Publisher('/plan', Plan, queue_size=10)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    sphere_sub = rospy.Subscriber('/sphere_params', SphereParams, sphere_callback)
    rate = rospy.Rate(10)  # 10 Hz
    plan = Plan()

    while not rospy.is_shutdown():
        distance_x, distance_y, distance_z = calculate_distance()
        if distance_x is not None and distance_y is not None and distance_z is not None:
            rospy.loginfo("Distance from camera to base: x=%f, y=%f, z=%f", distance_x, distance_y, distance_z)

            # Check if ball position is available
            if 'ball_position' in globals():
                try:
                    # Transform the ball position from camera frame to base frame
                    trans = tf_buffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
                    ball_position_base = tf2_geometry_msgs.do_transform_point(ball_position, trans)
                    ball_x_base = ball_position_base.point.x
                    ball_y_base = ball_position_base.point.y
                    ball_z_base = ball_position_base.point.z

                    rospy.loginfo("Ball position relative to base: x=%f, y=%f, z=%f", ball_x_base, ball_y_base, ball_z_base)

                    # Point 1
                    plan_point1 = Twist()
                    plan_point1.linear.x = 0.13
                    plan_point1.linear.y = -0.7
                    plan_point1.linear.z = 0.43
                    plan_point1.angular.x = 3.14
                    plan_point1.angular.y = 0
                    plan_point1.angular.z = 3.14
                    plan.points.append(plan_point1)

                    # Add the transformed ball position as point 2
                    plan_point2 = Twist()
                    plan_point2.linear.x = ball_x_base
                    plan_point2.linear.y = ball_y_base
                    plan_point2.linear.z = ball_z_base+0.01
                    plan_point2.angular.x = 3.14
                    plan_point2.angular.y = 0.0
                    plan_point2.angular.z = 3.14
                    plan.points.append(plan_point2)
                    
                    # Point 3
                    plan_point3 = Twist()
                    plan_point3.linear.x = -0.13
                    plan_point3.linear.y = -0.7
                    plan_point3.linear.z = 0.43
                    plan_point3.angular.x = 3.14
                    plan_point3.angular.y = 0.0
                    plan_point3.angular.z = 3.14
                    plan.points.append(plan_point3)

                    # Point 4
                    plan_point4 = Twist()
                    plan_point4.linear.x = -0.13
                    plan_point4.linear.y = -0.7
                    plan_point4.linear.z = ball_z_base+0.01
                    plan_point4.angular.x = 3.14
                    plan_point4.angular.y = 0.0
                    plan_point4.angular.z = 3.14
                    plan.points.append(plan_point4)

                    # Publish the plan
                    plan_pub.publish(plan)

                except tf2_ros.TransformException as e:
                    rospy.logerr("Failed to transform ball position to base frame: %s", e)

        rate.sleep()
