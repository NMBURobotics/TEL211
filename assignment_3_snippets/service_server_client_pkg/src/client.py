#!/usr/bin/env python

import rospy


class ClientNode():

    def __init__(self, *args):
        print("Constructing ClientNode")


if __name__ == '__main__':
    node = ClientNode()
    rospy.init_node('ClientNode', anonymous=True)
