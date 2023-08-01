#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL, SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Header
import math

class execute_square:

    def __init__(self):

        rospy.init_node('square_executer_node', anonymous = True)

        # ROS Publisher
        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)

        # ROS Subscriber
        self.pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # ROS Service clients
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Global variable to store position and orientation from the subscriber
        position = PoseStamped()

        # Wait to connect with FCU
        while not rospy.is_shutdown() and not self.is_connected():
            rospy.loginfo("Waiting for FCU connection...")
            rospy.sleep(1)

        # Change flight mode to guided
        if self.set_mode("GUIDED"):
            
            rospy.loginfo("Mode changed to GUIDED")
            rospy.sleep(2) # Wait for 2 seconds after changing mode

            self.arm_motors()
            rospy.sleep(4) # Sleep for 4 seconds after arming motors

            self.takeoff()
            rospy.sleep(2) # Sleep for 2 seconds after completing takeoff

            self.goto_waypoint(0, 5)
            rospy.sleep(1)
            self.turn_left(90)
            rospy.sleep(1)
            self.goto_waypoint(-5, 5)
            rospy.sleep(1)
            self.turn_left(90)
            rospy.sleep(1)
            self.goto_waypoint(-5, 0)
            rospy.sleep(1)
            self.turn_left(90)
            rospy.sleep(1)
            self.goto_waypoint(0, 0)
            rospy.sleep(1)
            self.turn_left(90)
            rospy.sleep(2)

            self.land()
            rospy.loginfo("Disarming")
            rospy.sleep(2)
            rospy.loginfo("Mission complete")
            rospy.signal_shutdown("mission completed")
        
        else:
            rospy.logerr("Failed to change mode to GUIDED")

    def pose_callback(self, msg):
        global position 
        position.header.stamp = rospy.Time.now()
        position.header.frame_id = 'map'
        position = msg
    
    def is_connected(self):
        state_msg = rospy.wait_for_message('/mavros/state', State, timeout = 5)
        return state_msg.connected
    
    def set_mode(self, mode):
        try:
            response = self.set_mode_service(0, mode)
            return response.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr("Mode change service call failed: " + str(e))
            return False
        
    def arm_motors(self):
        try:
            response = self.arm_service(True)
            if response.success:
                rospy.loginfo("Motors armed")
            else:
                rospy.logerr("Failed to arm motors")
        except rospy.ServiceException as e:
            rospy.logerr("Arming service call failed: " + str(e))

    def takeoff(self):
        try:
            response = self.takeoff_service(altitude=5.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            if response.success:
                rospy.loginfo("Takeoff successful")
                rospy.sleep(7)  # Wait for 7 seconds after takeoff
            else:
                rospy.logerr("Failed to takeoff.")
        
        except rospy.ServiceException as e:
            rospy.logerr("Takeoff service call failed: " + str(e))

    def goto_waypoint(self, x, y):
        rospy.loginfo("Going to: %d %d", x, y)

    def turn_left(self, angle_degrees):
        rospy.loginfo("Turning left")
        
        # Code for turning the drone left
        vel = TwistStamped()
        rate = rospy.Rate(10)
        global position
        quaternion = (position.pose.orientation.x, position.pose.orientation.y, position.pose.orientation.z,
                  position.pose.orientation.w)
        value = euler_from_quaternion(quaternion)
        current_yaw = value[2]
        desired_yaw = current_yaw + 1.57

        while abs(desired_yaw - current_yaw) > 0.02:
            vel.header = Header()
            vel.twist.angular.z = 0.1
            self.cmd_vel_pub.publish(vel)
            rate.sleep()

        vel.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)

        rospy.loginfo("Turn completed")

    def land(self):
        try:
            response = self.land_service(altitude=0.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            if response.success:
                rospy.loginfo("Landing now")
                rospy.sleep(12)
            else:
                rospy.logerr("Failed to send landing command.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))

if __name__ == '__main__':
    try:
        node = execute_square()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass