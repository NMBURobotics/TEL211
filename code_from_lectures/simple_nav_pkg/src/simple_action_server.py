#!/usr/bin/env python

import rospy
import actionlib
import simple_nav_pkg.msg
import math
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist
import tf.transformations

import tf2_ros
import tf2_geometry_msgs


class SimpleNavHelpers():
    def __init__(self, *args):
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def pose_euclidean_dist(self, a, b):

        return math.sqrt((a.position.x - b.position.x) ** 2 +
                         (a.position.y - b.position.y) ** 2 +
                         (a.position.z - b.position.z) ** 2)

    def get_curr_robot_pose(self):
        curr_robot_pose = PoseStamped()
        curr_robot_pose.header.frame_id = "odom"
        curr_robot_pose.header.stamp = rospy.Time().now()
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_link", rospy.Time().now(), rospy.Duration(1.0))
            curr_robot_pose.pose.position.x = transform.transform.translation.x
            curr_robot_pose.pose.position.y = transform.transform.translation.y
            curr_robot_pose.pose.position.z = transform.transform.translation.z
            curr_robot_pose.pose.orientation = transform.transform.rotation

        except (tf2_ros.TypeException, tf2_ros.NotImplementedException):
            rospy.loginfo("Failed to get current robot pose")
        return curr_robot_pose

    def transform_to_base_link(self, pose_stamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", "odom", rospy.Time().now(), rospy.Duration(1.0))
        except (tf2_ros.TypeException, tf2_ros.NotImplementedException):
            rospy.loginfo("Failed to get current robot pose")

        pose_transfromed = tf2_geometry_msgs.do_transform_pose(
            pose_stamped, transform)
        return pose_transfromed

    def clip(self, val, min_, max_):
        return min_ if val < min_ else max_ if val > max_ else val


class SimpleActionServer():

    def __init__(self, *args):
        self.feedback = simple_nav_pkg.msg.NavFeedback()
        self.result = simple_nav_pkg.msg.NavResult()

        self.action_server = actionlib.SimpleActionServer(
            "simple_nav", simple_nav_pkg.msg.NavAction, execute_cb=self.action_callback, auto_start=False)
        self.action_server.start()
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.simple_nav_helper = SimpleNavHelpers()

    def action_callback(self, goal):
        rospy.loginfo("recieved a goal")
        rospy.loginfo(goal.goal_pose)

        arrived_to_goal = False
        rate = rospy.Rate(15)

        inital_robot_pose = self.simple_nav_helper.get_curr_robot_pose()

        total_dist = self.simple_nav_helper.pose_euclidean_dist(
            inital_robot_pose.pose, goal.goal_pose.pose)

        while not arrived_to_goal and not rospy.is_shutdown():

            if self.action_server.is_preempt_requested():
                rospy.loginfo("Preempted simple_nav action server")
                self.action_server.set_preempted()
                arrived_to_goal = True
                break

            curr_robot_pose = self.simple_nav_helper.get_curr_robot_pose()
            curr_dist_to_goal = self.simple_nav_helper.pose_euclidean_dist(
                curr_robot_pose.pose, goal.goal_pose.pose)

            self.feedback.remaining_distance = curr_dist_to_goal
            self.action_server.publish_feedback(self.feedback)

            if curr_dist_to_goal < 0.5:
                rospy.loginfo("Arrived to goal")
                arrived_to_goal = True
                self.result.result = True
                self.action_server.set_succeeded()

            computed_velocity = Twist()
            # VERY SIMPLE PURE PURSUIT CONTROLLE

            robot_quat_exp = [curr_robot_pose.pose.orientation.x, curr_robot_pose.pose.orientation.y,
                              curr_robot_pose.pose.orientation.z, curr_robot_pose.pose.orientation.w]
            robot_euler = tf.transformations.euler_from_quaternion(
                robot_quat_exp)

            goal_quat_exp = [goal.goal_pose.pose.orientation.x, goal.goal_pose.pose.orientation.y,
                             goal.goal_pose.pose.orientation.z, goal.goal_pose.pose.orientation.w]
            goal_euler = tf.transformations.euler_from_quaternion(
                goal_quat_exp)

            robot_roll, robot_pitch, robot_yaw = robot_euler[0], robot_euler[1], robot_euler[2]
            goal_roll, goal_pitch, goal_yaw = goal_euler[0], goal_euler[1], goal_euler[2]

            err_local = [goal.goal_pose.pose.position.x - curr_robot_pose.pose.position.x,
                         goal.goal_pose.pose.position.y - curr_robot_pose.pose.position.y,
                         robot_yaw - goal_yaw]

            k1 = 1.0
            k2 = 5.0
            max_v = 0.5
            max_w = 0.5

            v_in = k1 * math.sqrt(err_local[0]**2 + err_local[1]**2)
            w_in = k2 * (math.atan2(err_local[1], err_local[0]) - robot_yaw)

            computed_velocity.linear.x = self.simple_nav_helper.clip(
                v_in, -max_v, max_v)

            computed_velocity.angular.z = self.simple_nav_helper.clip(
                w_in, -max_w, max_w)

            self.cmd_vel_pub.publish(computed_velocity)

            rate.sleep()


def main():
    rospy.init_node("simple_nav_node")
    action_server = SimpleActionServer()
    rospy.spin()


if __name__ == '__main__':
    main()
