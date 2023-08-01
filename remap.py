#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

class PositionRelayNode:
    
    def __init__(self):
        
        rospy.init_node('position_relay_node')

        self._subscriber = rospy.Subscriber('/vrpn_client_node/Real_Drone/pose', PoseStamped, self.callback_method)
        self._publisher = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)

        self.rate = rospy.Rate(25)  # Desired publishing rate (10 Hz)
        self.latest_message = PoseStamped()

    def callback_method(self, msg):
        
        self.latest_message = msg  # Overwrite the previous message with the most current one

    def relay_messages(self):
        
        while not rospy.is_shutdown():
            if self.latest_message is not None:
                self._publisher.publish(self.latest_message)
                self.latest_message = None  # Reset the latest message
            self.rate.sleep()

if __name__ == '__main__':
    
    try:
        node = PositionRelayNode()
        node.relay_messages()
    
    except rospy.ROSInterruptException:
        pass
