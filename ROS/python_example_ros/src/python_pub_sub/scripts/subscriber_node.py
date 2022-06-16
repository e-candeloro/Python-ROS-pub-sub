#!/usr/bin/env python3

# documentation: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# tutorial followed: https://youtu.be/MkiWj4VwZjc

import rospy
from std_msgs.msg import String

def callback(data):
    # callback function need as a parameter, the data object
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #data.data access the String msg received!
    
# listen function is executed in a permanent way till the node is stopped
def listen():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener_node', anonymous=True)

    # subscribe to the topic, specifing the data (String) and a callback function for when is received
    rospy.Subscriber("talking_topic", String, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.loginfo("Started subscriber_node, now listening to messages...")
        listen()
    except rospy.ROSInterruptException:
        pass