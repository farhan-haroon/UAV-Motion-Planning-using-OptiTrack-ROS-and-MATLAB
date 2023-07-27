#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL, SetMode
from mavros_msgs.srv import CommandBool
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
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Wait for the connection to FCU (Flight Controller Unit)
        while not rospy.is_shutdown() and not self.is_connected():
            rospy.loginfo("Waiting for FCU connection...")
            rospy.sleep(1)

        # Change flight mode to GUIDED
        if self.set_mode("GUIDED"):
            rospy.loginfo("Mode changed to GUIDED.")
            rospy.sleep(2)  # Wait for 2 seconds after changing mode
            self.arm_motors()
        else:
            rospy.logerr("Failed to change mode to GUIDED.")

    def is_connected(self):
        state_msg = rospy.wait_for_message('/mavros/state', State, timeout=5)
        return state_msg.connected

    def set_mode(self, mode):
        try:
            response = self.set_mode_service(0, mode)
            return response.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))
            return False

    def arm_motors(self):
        try:
            response = self.arm_service(True)
            if response.success:
                rospy.loginfo("Motors armed.")
                rospy.sleep(4)  # Wait for 2 seconds after arming motors
                self.takeoff()
            else:
                rospy.logerr("Failed to arm motors.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))

    def takeoff(self):
        try:
            response = self.takeoff_service(altitude=5.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            if response.success:
                rospy.loginfo("Takeoff successful.")
                rospy.sleep(7)  # Wait for 5 seconds after takeoff
                self.turn_left(90)
                rospy.sleep(2)  # Wait for 2 seconds after turning
                self.land()
                rospy.sleep(12)  # Wait for 10 seconds for the drone to land completely
                # self.disarm()
                rospy.signal_shutdown("commands_executed")
            else:
                rospy.logerr("Failed to takeoff.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))

    def turn_left(self, angle_degrees):
        # Code for turning the drone left
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

    def land(self):
        try:
            response = self.land_service(altitude=0.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            if response.success:
                rospy.loginfo("Landing command sent.")
            else:
                rospy.logerr("Failed to send landing command.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))


    """ def disarm(self):
        try:
            response = self.arm_service(False)
            if response.success:
                rospy.loginfo("Disarm command sent.")
            else:
                rospy.logerr("Failed to send disarm command.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e)) """

if __name__ == '__main__':
    try:
        node = DroneControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
