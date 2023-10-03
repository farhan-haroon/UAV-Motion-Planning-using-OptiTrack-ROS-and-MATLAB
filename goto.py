#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
import math 
import tf.transformations

class tester:

    def __init__(self):

        rospy.init_node('execute_circle_node', anonymous=True)
        self.rate = rospy.Rate(10)

        # Create a publisher for the position target
        self.pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # Create a subscriber for position data
        self.sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Global variable
        self.pose = PoseStamped()
        self.euler_yaw = Vector3()

        self.publish_position_target(4.0, 3, 2)

    def pose_callback(self, msg):
        self.pose = PoseStamped()
        self.pose = msg
        self.euler_yaw = self.quat_to_eul(msg.pose.orientation)

    def quat_to_eul(self, quaternion):
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        
        # Extract and return the yaw angle (in radians)
        return euler[2]

    def publish_position_target(self, x, y, z):

        goal_x = x
        goal_y = y
        goal_z = z
        c = 0

        # Create a PositionTarget message
        position_target_msg = PositionTarget()

        # Set the coordinate frame value to 8 (LOCAL_NED)
        position_target_msg.coordinate_frame = 8

        # Set the bitmask to 1987 to control position, velocity, acceleration, yaw, and yaw_rate
        position_target_msg.type_mask = 1987

        p_controller_linear = 0.8
        p_controller_vertical = 0.6
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            current_x = self.pose.pose.position.x
            current_y = self.pose.pose.position.y
            current_z = self.pose.pose.position.z
            current_yaw = self.euler_yaw

            if current_x and current_y and current_yaw and current_z is not None:
                
                while not rospy.is_shutdown():

                    current_x = self.pose.pose.position.x
                    current_y = self.pose.pose.position.y
                    current_z = self.pose.pose.position.z
                    current_yaw = self.euler_yaw

                    dist_xy = abs(math.sqrt(((goal_x - current_x) ** 2) +
                               ((goal_y - current_y) ** 2)))

                    if (dist_xy < 0.15 and dist_z < 0.05):
                        c = c + 1
                        break

                    dist_z = goal_z - current_z

                    linear_speed = dist_xy * p_controller_linear
                    vertical_speed = dist_z * p_controller_vertical

                    # set upper limit of linear velocity
                    linear_speed = min(linear_speed, 0.15)
                    # set lower limit of linear velocity
                    linear_speed = max(linear_speed, 0.02)

                    # set upper limit of linear velocity
                    vertical_speed = min(vertical_speed, 0.15)
                    # set lower limit of linear velocity
                    vertical_speed = max(vertical_speed, -0.15)

                    angle_to_goal = math.atan2(
                        goal_y - current_y, goal_x - current_x)

                    if (current_yaw < 0):
                        yaw = 6.28 - abs(current_yaw)

                    else:
                        yaw = current_yaw

                    if (angle_to_goal < 0):
                        angle_to_goal = 6.28 - abs(angle_to_goal)

                    delta_heading = math.atan2(
                        math.sin(angle_to_goal - yaw), math.cos(angle_to_goal - yaw))

                    if abs(angle_to_goal - yaw) < 0.02:
                        angular_speed = 0

                    if delta_heading <= 0.15 and delta_heading >=-0.15:
                        angular_speed = delta_heading

                    elif delta_heading > 0.25:
                        angular_speed = 0.25

                    elif delta_heading < -0.25:
                        angular_speed = -0.25

                    if abs(angular_speed) > 0.20:
                       linear_speed = 0.0
                       vertical_speed = 0.0

                    position_target_msg.velocity.x = linear_speed
                    position_target_msg.velocity.z = vertical_speed
                    position_target_msg.yaw_rate = angular_speed

                    self.pub.publish(position_target_msg)
                    rate.sleep()

                break

        position_target_msg.velocity.x = 0.0
        position_target_msg.velocity.z = 0.0
        position_target_msg.yaw_rate = 0.0

        if c > 0:
            self.pub.publish(position_target_msg)
            print("Done")
            rospy.signal_shutdown("done")
            c = 0

        else:
            rospy.signal_shutdown("kill")
            print("Keyboard interrupt!")


if __name__ == '__main__':
    try:
        node = tester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass