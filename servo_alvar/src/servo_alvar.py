#! /usr/bin/python
import roslib
roslib.load_manifest('tf')
roslib.load_manifest('actionlib')
roslib.load_manifest('move_base')

import sys

import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *

from geometry_msgs.msg import PointStamped, Pose, Quaternion, PoseStamped
from std_msgs.msg import String, Bool
import tf
from tf import transformations as tft

import numpy as np
from math import cos, sin, pi



class moveBasePractice:
    
    def __init__(self):
        rospy.init_node('servo_alvar')
        self.listener=tf.TransformListener()
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.simple_pub=rospy.Publisher('move_base_simple/goal', PoseStamped)
        self.marker="/ar_marker_"+str(marker_number)
        self.moveToMarker()
       
    def moveToMarker(self):  
        try:
            now=rospy.Time.now()
            self.listener.waitForTransform('/map', self.marker, now, rospy.Duration(10))
            trans, rot=self.listener.lookupTransform('/map', self.marker, now)  
            print "Have a TF frame"            
            self.move_base_client.wait_for_server()
            g = MoveBaseGoal()
            g.target_pose.header.frame_id = '/map'
            g.target_pose.header.stamp=rospy.Time.now()	
            #Add offset so that the PR2 sits right in front of the tag instead of running it over or getting the navigation stack stuck
            x=trans[0]-0.50                                  
            y=trans[1]
            z=trans[2]
            #Switch rotation around to be in the opposite direction        
            q1=np.array([rot[0], rot[1], rot[2], rot[3]])
            q1_con=tft.quaternion_conjugate(q1) #get the conjugate
            rot_y_axis=np.array([0.0*sin(pi/4), 1.0*sin(pi/4), 0.0*sin(pi/4), cos(pi/4)])
            q3=tft.quaternion_multiply(q1_con, q1)
            q4=tft.quaternion_multiply(rot_y_axis, q3)
            q_fin=tft.quaternion_multiply(q1, q4)

            g.target_pose.pose.position.x = x          
            g.target_pose.pose.position.y = y	        
            g.target_pose.pose.position.z = z
            g.target_pose.pose.orientation.x = q_fin[0]
            g.target_pose.pose.orientation.y = q_fin[1]
            g.target_pose.pose.orientation.z = q_fin[2]
            g.target_pose.pose.orientation.w = q_fin[3]
            
            self.move_base_client.send_goal(g)
            print "Made it this far, sent goal"
                     
            
            success_to_box=self.move_base_client.wait_for_result()
    
            if success_to_box:
                print "Movement to box successful"
                self.at_box=True
               
            else:
                print "Movement to box failed"
                self.at_box=False   
                

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
            print "TF Exception, try again"

if __name__=='__main__':
    if len(sys.argv)>1:
        marker_number=sys.argv[1]
    else:
        marker_number=1
    a=moveBasePractice()
    while not rospy.is_shutdown():
        rospy.spin()
