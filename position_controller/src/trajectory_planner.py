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
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64

class trajectoryPlanner(object):

    def __init__(self, name):

        self._name = name
        self.pub_sp = rospy.Publisher("/move_base/TebLocalPlannerROS/local_plan", Path, queue_size=1)

        self.plan = Path()
#        self.plan.poses[0].pose.position.x = 0.0
#        self.plan.poses[0].pose.position.y = 0.0
#        self.plan.poses[0].pose.position.z = 0.0
#        self.plan.poses[0].pose.orientation.x = 0.0
#        self.plan.poses[0].pose.orientation.y = 0.0
#        self.plan.poses[0].pose.orientation.z = 0.0
#        self.plan.poses[0].pose.orientation.w = 1.0

        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Start Trajectory planner")

    def cleanup(self):
        '''
        @brief destructor
        '''
        rospy.loginfo("Stop Trajectory planner")
        rospy.sleep(0.5)

    def setPlan(self, path):
        print path
        yaw = path[2] * math.pi / 180.0
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

        self.plan.poses[0].pose.position.x = path[0] 
        self.plan.poses[0].pose.position.y = path[1]
        self.plan.poses[0].pose.position.z = 0.0
        self.plan.poses[0].pose.orientation.x = quat[0]
        self.plan.poses[0].pose.orientation.y = quat[1]
        self.plan.poses[0].pose.orientation.z = quat[2]
        self.plan.poses[0].pose.orientation.w = quat[3]

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", help="Broadcast Rate", type=float, default=5.0)
    parser.add_argument("--points", help="Trajectory points", type=int, default=1)
    args, unknown = parser.parse_known_args()
    
    try:
        rospy.init_node("Trajectory_planner")

        tp = trajectoryPlanner(rospy.get_name())
        rate = rospy.Rate(args.rate)
        i = 0
        square = [[3.0,0.0,0.0],[3.0,0.0,90.0],[3.0,3.0,90.0],[3.0,3.0,180.0]]

        while not rospy.is_shutdown():
            if (i < len(square)):
                tp.setPlan(square[i])
                tp.pub_sp.publish(tp.plan)
            # Debug
                print i
                i += 1
                rate.sleep()
            else:
                break
        #rate.sleep()


    except rospy.ROSInterruptException:
        pass
    # rospy.spin()

