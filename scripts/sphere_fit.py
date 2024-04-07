#!/usr/bin/env python3

import rospy
import numpy as np
from robot_vision_lectures.msg import XYZarray, SphereParams
from geometry_msgs.msg import Point

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.prev_value = None

    def __call__(self, value):
        if self.prev_value is None:
            filtered_value = value
        else:
            filtered_value = self.alpha * value + (1 - self.alpha) * self.prev_value
        self.prev_value = filtered_value
        return filtered_value

class SphereFitNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('sphere_fit_node', anonymous=True)
        rospy.loginfo("Sphere Fit Node initialized")
        
        # Subscriber for receiving XYZ points
        self.subscriber = rospy.Subscriber('/xyz_cropped_ball', XYZarray, self.xyz_callback, queue_size=1)
        
        # Publisher for sending sphere parameters
        self.publisher = rospy.Publisher('/sphere_params', SphereParams, queue_size=1)
        
        # Set the publishing rate
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Variable to store received XYZ points
        self.xyz_points = None
        
        # Low-pass filters for each parameter
        self.xc_filter = LowPassFilter(alpha=0.05)
        self.yc_filter = LowPassFilter(alpha=0.05)
        self.zc_filter = LowPassFilter(alpha=0.05)
        self.radius_filter = LowPassFilter(alpha=0.05)

    # Callback function for processing received XYZ points
    def xyz_callback(self, msg):
        rospy.loginfo("Received XYZ points")
        self.xyz_points = np.array([[point.x, point.y, point.z] for point in msg.points])

    # Function to fit a sphere to the received XYZ points
    def fit_sphere(self):
        # Exception handler for error in lab 5
        try:
            # Check if sufficient XYZ points are available
            if self.xyz_points is None or len(self.xyz_points) < 4:
                rospy.logwarn("Insufficient XYZ points received.")
                return None
            
            # Another problem, below is solution
            # Filter out points with shape other than (3,)
            valid_points = []
            for point in self.xyz_points:
                if len(point) != 3:
                    rospy.logwarn("Invalid XYZ point received: %s", point)
                    continue
                valid_points.append(point)
    
            # Check if there are enough valid points
            if len(valid_points) < 4:
                rospy.logwarn("Insufficient valid XYZ points received.")
                return None
    
            # Ensure all points have the same dimensionality
            max_dim = 3
            xyz_points_fixed = np.zeros((len(valid_points), max_dim))
            for i, point in enumerate(valid_points):
                xyz_points_fixed[i, :len(point)] = point
    
            # Prepare matrices for solving the least squares problem
            A = np.column_stack((2*xyz_points_fixed, np.ones(len(valid_points))))
            B = np.sum(xyz_points_fixed**2, axis=1)
    
            # Solve the least squares problem to obtain sphere parameters
            P = np.linalg.lstsq(A, B, rcond=None)[0]
    
            # Calculate sphere center coordinates and radius
            x_c, y_c, z_c = P[0], P[1], P[2]
            radius = np.sqrt(x_c**2 + y_c**2 + z_c**2 + P[3])
    
            return x_c, y_c, z_c, radius
        except np.linalg.LinAlgError as e:
            rospy.logerr("Error occurred during sphere fitting: %s", e)
            return None
        
    # Main function to run the node
    def run(self):
        rospy.loginfo("Sphere Fit Node is running")
        while not rospy.is_shutdown():
            rospy.loginfo("Fitting sphere")
            sphere_params_msg = SphereParams()
            
            # Fit a sphere to the received XYZ points
            params = self.fit_sphere()

            if params is None:
                rospy.logwarn("Sphere fitting failed")
                continue

            # Apply low-pass filters to parameters
            filtered_xc = self.xc_filter(params[0])
            filtered_yc = self.yc_filter(params[1])
            filtered_zc = self.zc_filter(params[2])
            filtered_radius = self.radius_filter(params[3])

            rospy.loginfo("Sphere parameters: x_c=%f, y_c=%f, z_c=%f, radius=%f",
                          filtered_xc, filtered_yc, filtered_zc, filtered_radius)

            # Populate the SphereParams message with calculated parameters
            sphere_params_msg.xc = filtered_xc
            sphere_params_msg.yc = filtered_yc
            sphere_params_msg.zc = filtered_zc
            sphere_params_msg.radius = filtered_radius

            # Publish the SphereParams message
            self.publisher.publish(sphere_params_msg)
        
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize and run the SphereFitNode
        sphere_fit_node = SphereFitNode()
        sphere_fit_node.run()
    except rospy.ROSInterruptException:
        pass
