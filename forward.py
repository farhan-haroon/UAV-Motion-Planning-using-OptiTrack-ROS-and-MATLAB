#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

def send_forward_waypoint():
    rospy.init_node('forward_waypoint_publisher')
    rate = rospy.Rate(10)  # Update rate (Hz)

    setpoint_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'  # Use 'map' frame to indicate local coordinates

        # Set the desired forward distance in meters
        forward_distance = 10.0

        # Set the desired position relative to the current position
        pose.pose.position.x = - 10.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 10.0

        # Set the desired orientation to face forward (keep the current orientation)
        orientation = Quaternion(*quaternion_from_euler(0, 0, 0))  # Roll, Pitch, Yaw (in radians)

        pose.pose.orientation = orientation

        setpoint_publisher.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_forward_waypoint()
    except rospy.ROSInterruptException:
        pass
