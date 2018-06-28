#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from move_base_msgs.msg import MoveBaseActionFeedback


def initialize():
    global pose_pub
    rospy.init_node('pose_rerouter', anonymous=True)
    rospy.loginfo('pose rerouter node crearted')
    rospy.Subscriber('/pr2_move_base/feedback', MoveBaseActionFeedback, pose_callback)
    rospy.loginfo('Subscribing to feedback')
    pose_pub = rospy.Publisher('/robot_pose', Pose, queue_size=10)
    rospy.spin()


def pose_callback(base_position):
    global pose_pub
    pose_pub.publish(base_position.feedback.base_position.pose)

def main():
    try:
        initialize()
        r = rospy.Rate(30)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
