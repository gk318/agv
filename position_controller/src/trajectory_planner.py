#!/usr/bin/env python

"""
This script publishes a sample trajectory
"""

import argparse
import time
import math

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Bool

class trajectoryPlanner(object):

    def __init__(self, name):

        self._name = name
        self.pub_sp = rospy.Publisher("/move_base/TebLocalPlannerROS/global_plan", Path, queue_size=1)

        self.plan = Path()
        self.plan.header.frame_id = "agv_base_footprint"

        self.pose = PoseStamped()

        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Start Trajectory planner")

    def cleanup(self):
        '''
        @brief destructor
        '''
        rospy.loginfo("Stop Trajectory planner")
        rospy.sleep(0.5)

    def poseFromListItem(self, list_item):
        pose = PoseStamped()
        sq_yaw = list_item[2] * math.pi / 180.0
        sq_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, sq_yaw)

        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = list_item[0]
        pose.pose.position.y = list_item[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = sq_quat[0] 
        pose.pose.orientation.y = sq_quat[1] 
        pose.pose.orientation.z = sq_quat[2] 
        pose.pose.orientation.w = sq_quat[3]
        self.plan.poses.append(pose)


if __name__ == "__main__":        
    
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", help="Broadcast Rate", type=float, default=1.0)
    parser.add_argument("--points", help="Trajectory points", type=int, default=1)
    args, unknown = parser.parse_known_args()
    
    try:
        rospy.init_node("Trajectory_planner")

        tp = trajectoryPlanner(rospy.get_name())
        rate = rospy.Rate(args.rate)
        square = [[4.0,0.0,0.0],[4.0,0.0,90.0],[4.0,2.0,90.0],[4.0,2.0,180.0], [0.0,2.0,180.0],[0.0, 2.0, 270.0], [0.0,0.0,270.0], [0.0, 0.0, 360.0]]
	triangle = [[0.0,0.0,135.0],[3.0,3.0,135.0], [3.0,3.0,180.0],[0.0,3.0,180.0],[0.0,3.0,270.0],[0.0,0.0,270.0],[0.0,0.0,0.0]]
        abstract = [[0.0,0.0,0.0],[1.0,0.0,0.0],[1.0,0.0,90.0],[1.0,3.0,90.0],[1.0,3.0,0.0],[1.5,3.0,0.0],[1.0,3.0,90.0],[1.0,3.0,90.0],[1.0,3.0,90.0]]
        for list_item in square:
            tp.poseFromListItem(list_item)
        while not rospy.is_shutdown():
            tp.pub_sp.publish(tp.plan)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    rospy.spin()

