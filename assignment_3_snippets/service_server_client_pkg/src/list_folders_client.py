#!/usr/bin/env python

import rospy
from service_server_client_pkg.srv import ListFolders, ListFoldersResponse, ListFoldersRequest


class ListFoldersClientNode():

    def __init__(self, *args):
        print("Constructing ListFoldersClientNode")
        rospy.init_node('ListFoldersClientNode', anonymous=True)

    def list_folders_client(self, path):
        rospy.wait_for_service('list_folders')

        try:
            service = rospy.ServiceProxy('list_folders', ListFolders)
            response = service(path)
            return response
        except rospy.ServiceException as e:
            rospy.loginfo('Failed to call list_folders', 'service')
            rospy.loginfo('Exection is %s' % e)


if __name__ == '__main__':
    node = ListFoldersClientNode()
    print(node.list_folders_client('/home/tel211/'))
