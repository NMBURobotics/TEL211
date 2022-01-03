#!/usr/bin/env python

from std_msgs.msg._String import String
import rospy

class TalkerNode():
    
    def __init__(self, *args):
        print("Constructing")
        self.counter = 0
    
    def publish(self):
        print("I am publishing ", self.counter)
        self.counter += 1
        
if __name__ == '__main__':
    node = TalkerNode()
    rospy.init_node('TalkerNode', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()
    print("Shutting down TalkerNode")
        