#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist

def matrix_to_translation_rotation(matrix):
    """
    Extract translation and rotation (as a quaternion) from a transformation matrix.
    """
    translation = matrix[:3, 3]
    rotation = tf.transformations.quaternion_from_matrix(matrix)
    return translation, rotation

def compute_velocity(previous_transform, current_transform, delta_time):
    """
    Compute linear and angular velocity given two transformation matrices and the time difference.
    """
    prev_translation, prev_rotation = matrix_to_translation_rotation(previous_transform)
    curr_translation, curr_rotation = matrix_to_translation_rotation(current_transform)

    # Compute linear velocity (difference in translation)
    linear_velocity = (curr_translation - prev_translation) / delta_time

    # Compute angular velocity (difference in rotation)
    delta_rotation = tf.transformations.quaternion_multiply(
        curr_rotation, tf.transformations.quaternion_inverse(prev_rotation))
    
    
    # Convert delta_rotation quaternion to angular velocity vector
    angular_velocity = 2 * np.array(delta_rotation[:3]) / delta_time

    return linear_velocity, angular_velocity

class VelocityCalculator:
    def __init__(self, target_frame, source_frame="world"):
        self.listener = tf.TransformListener()
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.prev_transform = None
        self.prev_time = None

        #self.velocity_pub = rospy.Publisher(f"{self.target_frame}_velocity", Twist, queue_size=10)

    def compute_and_publish_velocity(self):
        #print("computing")
        try:
            (trans, rot) = self.listener.lookupTransform(self.source_frame, self.target_frame, rospy.Time(0))
            current_transform = tf.transformations.quaternion_matrix(rot)
            current_transform[:3, 3] = trans
            # print(trans)
            current_time = rospy.Time.now().to_sec()

            if self.prev_transform is not None:
                delta_time = current_time - self.prev_time
                linear_velocity, angular_velocity = compute_velocity(self.prev_transform, current_transform, delta_time)
                
                # # Publish the velocity
                # velocity_msg = Twist()
                # velocity_msg.linear.x, velocity_msg.linear.y, velocity_msg.linear.z = linear_velocity
                # velocity_msg.angular.x, velocity_msg.angular.y, velocity_msg.angular.z = angular_velocity
                # self.velocity_pub.publish(velocity_msg)
                print(self.target_frame + " velocity :", linear_velocity)

            # Update the previous transform and time
            self.prev_transform = current_transform
            self.prev_time = current_time

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

def main():
    rospy.init_node('frame_velocity_calculator')

    #target_frame = "dinova1_tool_frame"
    source_frame = "world"

    #"chasis_link", "forearm_link", "upper_wrist_link", "tool_frame"
    velocity_calculator1 = VelocityCalculator("dinova1_chassis_link", source_frame)
    velocity_calculator2 = VelocityCalculator("dinova1_forearm_link", source_frame)
    velocity_calculator3 = VelocityCalculator("dinova1_upper_wrist_link", source_frame)
    velocity_calculator4 = VelocityCalculator("dinova1_tool_frame", source_frame)

    rate = rospy.Rate(50)  # 10 Hz
    while not rospy.is_shutdown():
        velocity_calculator1.compute_and_publish_velocity()
        velocity_calculator2.compute_and_publish_velocity()
        velocity_calculator3.compute_and_publish_velocity()
        velocity_calculator4.compute_and_publish_velocity()
        rate.sleep()

if __name__ == '__main__':
    main()
