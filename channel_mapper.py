#!/usr/bin/env python

import rospy
import rosbag
from mavros_msgs.msg import RCIn, OverrideRCIn

def convert_and_save_bag(input_bag_filename, output_bag_filename):
    with rosbag.Bag(output_bag_filename, 'w') as output_bag:
        for topic, msg, t in rosbag.Bag(input_bag_filename).read_messages():
            if topic == '/mavros/rc/in':
                override_msg = OverrideRCIn()
                
                # Pad the channels tuple with zeros if it has less than 18 items
                padded_channels = msg.channels + (0,) * (18 - len(msg.channels))
                
                # Fill the override_msg with the padded channels tuple
                override_msg.channels = padded_channels
                
                # Write the converted message to the output bag
                output_bag.write('/mavros/rc/override', override_msg, t)
            else:
                output_bag.write(topic, msg, t)

if __name__ == '__main__':
    rospy.init_node('rc_converter_node')
    
    input_bag_filename = 'rc_in_2023-08-10-16-41-26.bag'  # Replace with your input bag file
    output_bag_filename = 'output.bag'  # Replace with your output bag file
    
    convert_and_save_bag(input_bag_filename, output_bag_filename)
