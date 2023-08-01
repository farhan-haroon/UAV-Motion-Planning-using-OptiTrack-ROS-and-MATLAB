#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL, SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped

class execute_circle:

    def __init__(self):

        rospy.init_node('execute_circle_node', anonymous=True)
        self.rate = rospy.Rate(10)

        # Create a publisher for the position target
        self.pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # Create a subscriber for position data
        self.sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Global variable
        self.pose = PoseStamped()

        # ROS service clients
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)    

        # Wait for the connection to FCU (Flight Controller Unit)
        while not rospy.is_shutdown() and not self.is_connected():
            rospy.loginfo("Waiting for FCU connection...")
            rospy.sleep(2)

        # Change flight mode to GUIDED
        if self.set_mode("GUIDED"):
            rospy.loginfo("Mode changed to GUIDED.")
            rospy.sleep(2)  # Wait for 2 seconds after changing mode
            self.arm_motors()
        else:
            rospy.logerr("Failed to change mode to GUIDED.")

    def pose_callback(self, msg):
        self.pose = PoseStamped()
        self.pose = msg
    
    def is_connected(self):
        state_msg = rospy.wait_for_message('/mavros/state', State, timeout=60)
        rospy.loginfo("Connected with FCU")
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
                rospy.loginfo("Motors armed.")
                rospy.sleep(4)  # Wait for 4 seconds after arming motors
                self.takeoff()
            else:
                rospy.logerr("Failed to arm motors.")
        except rospy.ServiceException as e:
            rospy.logerr("Arming service call failed: " + str(e))
        
    def takeoff(self):

        try:
            response = self.takeoff_service(altitude=5.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            
            if response.success:
                rospy.loginfo("Takeoff successful.")
                rospy.sleep(7)  # Wait for 7 seconds after takeoff

                # Execute circle
                self.publish_position_target()

        except rospy.ServiceException as e:
            rospy.logerr("Takeoff service call failed: " + str(e))

    def land(self):
        try:
            response = self.land_service(altitude=0.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            if response.success:
                rospy.loginfo("Landing command sent.")
                rospy.sleep(10)
            else:
                rospy.logerr("Failed to send landing command.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))

    
    
    def publish_position_target(self):

        while not rospy.is_shutdown() and not(self.pose.pose.position.x < 0.5 and self.pose.pose.position.x > -0.5 and self.pose.pose.position.y < -0.1 and self.pose.pose.position.y > -0.5):
            
            rospy.loginfo("Traversing circle")
            
            # Create a PositionTarget message
            position_target_msg = PositionTarget()

            # Set the coordinate frame value to 8 (LOCAL_NED)
            position_target_msg.coordinate_frame = 8

            # Set the bitmask to 1987 to control position, velocity, acceleration, yaw, and yaw_rate
            position_target_msg.type_mask = 1987

            # Set the desired velocity in the x-axis to 0.2 m/s
            position_target_msg.velocity.x = 0.4

            # Set the desired yaw rate to 0.1 rad/s
            position_target_msg.yaw_rate = 0.2

            # Publish the message
            self.pub.publish(position_target_msg)

            rospy.loginfo("X pos: %.3f",  self.pose.pose.position.x)
            rospy.loginfo("Y pos: %.3f",  self.pose.pose.position.y)

            # Sleep to achieve the desired rate
            self.rate.sleep()

        
        # Stopping the drone
        position_target_msg.velocity.x = 0.0
        position_target_msg.yaw_rate = 0.0
        self.pub.publish(position_target_msg)

        rospy.loginfo("Circle formed")
        rospy.sleep(2)
        
        # Land the drone
        rospy.loginfo("Landing")
        self.land()

        rospy.loginfo("Mission complete")
        rospy.sleep(2)
        rospy.signal_shutdown("completed")

if __name__ == '__main__':
    try:
        node = execute_circle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
