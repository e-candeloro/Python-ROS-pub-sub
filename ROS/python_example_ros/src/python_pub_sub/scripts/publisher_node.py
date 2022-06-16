#!/usr/bin/env python3

# See documentation on ROS Wiki: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# Tutorial followed: https://youtu.be/yqYvMEYJoTk

import rospy
from std_msgs.msg import String

def talk():
    # instatiate the publisher with a topic name, a data type and queue size
    pub = rospy.Publisher("talking_topic", String, queue_size=10)
    
    # initialize the node and assign a name to it.
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("publisher_node", anonymous=True)
    # set a pub rate in hertz (in this case 1 Hertz = 1 sec)
    rate = rospy.Rate(1)
    
    #log when node starts (really useful for debugging)
    rospy.loginfo("Started publisher_node, now publishing messages...")
    
    # while the node is working execute this loop
    
    while not rospy.is_shutdown():
        msg = "Hello, it's the pub speaking: %s " %rospy.get_time()
        rospy.loginfo(msg)
        pub.publish(msg) #publish the message
        rate.sleep() #sleep for the amount setted with Rate

if __name__ == '__main__':
    try:
        talk()
    except rospy.ROSInterruptException:
        pass