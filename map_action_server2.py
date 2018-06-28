#! /usr/bin/env python

import rospy
import actionlib
import actionlib_tutorials.msg
import move_base_msgs.msg
import tf
import numpy as np
import math as m

def slope(x1, y1, x2, y2):
    m = (y2-y1)/(x2-x1)
    return m

class MoveBaseAction(object):
    # create messages that are used to publish feedback/result
    _feedback = move_base_msgs.msg.MoveBaseFeedback()
    _result = move_base_msgs.msg.MoveBaseResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, move_base_msgs.msg.MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(30)
        success = True
        
        # # append the seeds for the fibonacci sequence
        # self._feedback.sequence = []
        # self._feedback.sequence.append(0)
        # self._feedback.sequence.append(1)
        
        # publish info to the console for the user
        # rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # ============= Main loop - Turn ===========

        destination = [goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,goal.target_pose.pose.position.z]
        start_pose_position = [20,30,0]
        start_pose_orientation = [0,0,0,1]

        angle = slope(start_pose_position[0],start_pose_position[1],destination[0],destination[1])

        self._feedback.base_position.pose.position.x = start_pose_position[0]
        self._feedback.base_position.pose.position.y = start_pose_position[1]
        self._feedback.base_position.pose.position.z = start_pose_position[2]

        self._feedback.base_position.pose.orientation.x = start_pose_orientation[0]
        self._feedback.base_position.pose.orientation.y = start_pose_orientation[1]
        self._feedback.base_position.pose.orientation.z = start_pose_orientation[2]
        self._feedback.base_position.pose.orientation.w = start_pose_orientation[3]

        current_quat = [self._feedback.base_position.pose.orientation.x,self._feedback.base_position.pose.orientation.y,self._feedback.base_position.pose.orientation.z,self._feedback.base_position.pose.orientation.w]
        current_euler = tf.transformations.euler_from_quaternion(current_quat)

        target_yaw = m.atan2((destination[0] - start_pose_position[0]), (destination[1] - start_pose_position[1]))

        theta_tolerance = 0.01
        distance_tolerance = 0.01

        while (abs(current_euler[2] - target_yaw) > theta_tolerance):
            if current_euler[2] > target_yaw:
                current_euler = list(current_euler)
                current_euler[2]-= 0.01
            elif current_euler[2] < target_yaw:
                current_euler = list(current_euler)
                current_euler[2]+= 0.01
            else:
                print ''

            [self._feedback.base_position.pose.orientation.x,self._feedback.base_position.pose.orientation.y,self._feedback.base_position.pose.orientation.z,self._feedback.base_position.pose.orientation.w] = tf.transformations.quaternion_from_euler(current_euler[0], current_euler[1], current_euler[2])
            self._as.publish_feedback(self._feedback)

            current_quat = [self._feedback.base_position.pose.orientation.x,self._feedback.base_position.pose.orientation.y,self._feedback.base_position.pose.orientation.z,self._feedback.base_position.pose.orientation.w]
            current_euler = tf.transformations.euler_from_quaternion(current_quat)
            r.sleep()
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break



        while (np.linalg.norm([destination[0] - self._feedback.base_position.pose.position.x, destination[1] - self._feedback.base_position.pose.position.y]) >= distance_tolerance):
            self._feedback.base_position.pose.position.x += 0.01*(destination[0] - start_pose_position[0])
            self._feedback.base_position.pose.position.y += 0.01*(destination[1] - start_pose_position[1])
            self._as.publish_feedback(self._feedback)
            r.sleep()
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

          
        if success:
            self._result = self._feedback
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('pr2_move_base')
    server = MoveBaseAction(rospy.get_name())
    rospy.spin()