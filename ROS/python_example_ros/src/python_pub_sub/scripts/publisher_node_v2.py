#!/usr/bin/env python3

"""
This script uses the ROSPublisher class in the 
ros_publisher.py file to publish a message to a topic.
"""

from ros_publisher import ROSPublisher as rospub


if __name__ == '__main__':

    pub = rospub(node_name="publisher_node_v2",
                 topic="talking_topic", data="String", queue=10, signal=True)
    # pay attention to the flag signal=True, used only with the publish_recur method!

    # pub.publish_recur(message="Hello!", rate_pub=1)
    # pub.publish_recur publishes same message at a given rate (in Hz) till it is killed by the user

    for i in range(101):
        pub.publish_once(message=f"hello for the {i}th time!")
        # pub.publish_once publishes a message once but need to be killed using the method pub.killnode
        
    pub.killnode()
    print("Terminated, thanks to the pub.killnode() function at the end of this script :) ")
