#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import math

class DroneControlNode:
    def __init__(self):
        rospy.init_node('drone_control_node', anonymous=True)

        # ROS publishers
        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)

        # ROS service clients
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        # Wait for the connection to FCU (Flight Controller Unit)
        while not rospy.is_shutdown() and not self.is_connected():
            rospy.loginfo("Waiting for FCU connection...")
            rospy.sleep(1)

        # Call the takeoff service
        if self.takeoff():
            rospy.loginfo("Takeoff successful.")
            rospy.sleep(5)  # Wait for 5 seconds after takeoff
            self.turn_left(90)
        else:
            rospy.logerr("Failed to takeoff.")

    def is_connected(self):
        state_msg = rospy.wait_for_message('/mavros/state', State, timeout=5)
        return state_msg.connected

    def takeoff(self):
        try:
            response = self.takeoff_service(altitude=5.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))
            return False

    def turn_left(self, angle_degrees):
        angular_speed = math.radians(30)  # 30 degrees per second
        angular_distance = math.radians(angle_degrees)  # Convert angle to radians
        twist_msg = TwistStamped()
        twist_msg.header = Header()
        twist_msg.twist.angular.z = angular_speed

        # Calculate the time needed to perform the turn
        turn_time = abs(angular_distance) / angular_speed

        start_time = rospy.Time.now().to_sec()
        current_time = rospy.Time.now().to_sec()

        while current_time - start_time < turn_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(0.1)
            current_time = rospy.Time.now().to_sec()

        # Stop the drone after turning
        twist_msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        rospy.loginfo("Turn completed.")

if __name__ == '__main__':
    try:
        node = DroneControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
