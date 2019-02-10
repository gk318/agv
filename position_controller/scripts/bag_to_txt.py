#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract odom from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    #parser.add_argument("odom_topic", help="odom topic.")
    parser.add_argument("plan_topic", help="Global plan topic.")    

    args = parser.parse_args()

    print "Extract plan from %s on topic %s into %s" % (args.bag_file,
                                                          args.plan_topic, args.output_dir)

    bag = rosbag.Bag(args.bag_file, "r")
    #f_odom= open(args.output_dir+"odom.txt","w+")
    f_plan = open(args.output_dir+"plan.txt", "w+")

    odom = Odometry()
    plan = Path()
    count = 0
    #for topic, msg, t in bag.read_messages(topics=[args.odom_topic]):
    for topic, msg, t in bag.read_messages(topics=[args.plan_topic]):
        
        # ODOM extraction
        #x = msg.pose.pose.position.x
        #y = msg.pose.pose.position.y
        #orientation_quat = msg.pose.pose.orientation
        #orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        #(_,_,yaw) = euler_from_quaternion(orientation_list)
        #print("x = {}, y = {}, yaw = {}".format(x, y, yaw))
        #print "Wrote Pose %i" % count

        # PLAN extraction
        if (count < len(msg.poses)):

            pose = msg.poses[count]

            print pose

            x = pose.pose.position.x
            y = pose.pose.position.y
            orientation_quat = pose.pose.orientation
            orientation_list = [orientation_quat.x, orientation_quat.y, 
            orientation_quat.z, orientation_quat.w]
            (_,_,yaw) = euler_from_quaternion(orientation_list)
            #print("x = {}, y = {}, yaw = {}".format(x, y, yaw))
            print "Wrote Pose %i" % count


            # TIME extraction
            #time = round(t.to_time()/1E9,6)
            #time_nsecs = abs(t.nsecs) / 1000
            time_nsecs = t.nsecs

            f_plan.write("poses " + str(count) + "\t" + str(t.secs) + "." + str(t.nsecs) + "\t" +
                str(x) + "\t" + str(y) + "\t" + str(yaw) + "\n")
    
       #f.write(str(t.to_nsec()
        #/1E3)+"\t"+("frame%06i.png" % count)+"\n")
    
        #f_odom.write(str(t.secs)+"."+str(time_nsecs) + "\t" + str(x)+ "\t" + str(y) + "\t" + str(yaw) + "\n")


            count += 1

    bag.close()
    #f_odom.close() 
    f_plan.close()

    return

if __name__ == '__main__':
    main()

