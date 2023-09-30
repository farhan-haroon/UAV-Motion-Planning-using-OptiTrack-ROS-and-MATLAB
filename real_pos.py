#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseStamped

obj = PoseStamped()

def pose_callback(data):

    global obj
    # This function is called every time a new PoseStamped message is received
    # Publish the received position to another topic
    obj.pose.position.x = data.pose.position.x
    obj.pose.position.y = data.pose.position.y * -1
    obj.pose.position.z = data.pose.position.z

    obj.pose.orientation = data.pose.orientation

    print(
        "X: ", obj.pose.position.x,
        "Y: ", obj.pose.position.y,
        "Z: ", obj.pose.position.z,
    )

def method():

    global obj
    rate = rospy.Rate(20)  # Adjust the rate as needed (in Hz)

    rospy.Subscriber('/vrpn_client_node/drone/pose', PoseStamped, pose_callback)
    while not rospy.is_shutdown():

        pub = rospy.Publisher("/drone_pose/NED", PoseStamped, queue_size=10)
        pub.publish(obj)
        rate.sleep()

if __name__ == '__main__':
    
    rospy.init_node('pose_subscriber_node', anonymous=True)
    method()