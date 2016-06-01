#!/usr/bin/env python
import sys, threading
import random as r
import xml.etree.ElementTree as etree

import rospy
from std_msgs.msg import Float32, Float64
from collections import deque

from poses import Poses

class JimmyController(threading.Thread):
    roll = 0.0
    pan = 0.0
    then = 0

    def __init__(self):
        # self.yaw_sub = rospy.Subscriber('/head/cmd_pose_yaw', Float32, self.yaw_callback)
        # self.pitch_sub = rospy.Subscriber('/head/cmd_pose_pitch', Float32, self.pitch_callback)

        # List of recognized joints (note, left_sho_pitch servo doesn't work)
        self.head_pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=10)        
        self.head_tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=10)
        self.right_sho_pitch = rospy.Publisher('/right_sho_pitch/command', Float64, queue_size=10)
        self.right_sho_roll = rospy.Publisher('/right_sho_roll/command', Float64, queue_size=10)
        self.right_elbow = rospy.Publisher('/right_elbow/command', Float64, queue_size=10)
        self.left_sho_pitch = rospy.Publisher('/left_sho_pitch/command', Float64, queue_size=10)
        self.left_sho_roll = rospy.Publisher('/left_sho_roll/command', Float64, queue_size=10)
        self.left_elbow = rospy.Publisher('/left_elbow/command', Float64, queue_size=10)

        self.joints = {'R_ELBOW': self.right_elbow,
            'R_SHO_PITCH': self.right_sho_pitch,
            'R_SHO_ROLL': self.right_sho_roll,
            'L_ELBOW': self.left_elbow,
            'L_SHO_PITCH': self.left_sho_pitch,
            'L_SHO_ROLL': self.left_sho_roll,
            'HEAD_PAN': self.head_pan_pub,
            'HEAD_TILT': self.head_tilt_pub}

        # Load from .pagelist file
        tree = etree.parse('/home/mathias/Downloads/WinRME/Chair-Poses.pagelist')
        # tree = etree.parse('/home/mathias/Projects/jimmy_ros/src/arbotix_controller/src/PositionSequence.pagelist')
        self.pages = tree.findall('.//PageClass')
        self.page_length = len(self.pages)

        self.action = []    

        # print "Pose classes: %s" % pose_classes
        # R_SHO_PITCH = [[int(pcx.find('R_SHO_PITCH').text) for pcx in pc] for pc in pose_classes]

        # self.poses = Poses(self.pages[0])    # Only pick the first 'page'
        # rospy.loginfo(self.poses.getPoses())

        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(20)
        
    def run(self):
        rospy.loginfo("Setting up JimmyController ...")

        # joints = {'R_ELBOW': self.right_elbow,
        #     'R_SHO_PITCH': self.right_sho_pitch,
        #     'R_SHO_ROLL': self.right_sho_roll,
        #     'L_ELBOW': self.left_elbow,
        #     'L_SHO_PITCH': self.left_sho_pitch,
        #     'L_SHO_ROLL': self.left_sho_roll,
        #     'HEAD_PAN': self.head_pan_pub,
        #     'HEAD_TILT': self.head_tilt_pub}

        # self.poses = Poses(pages[0])    # Only pick the first 'page'
        # rospy.loginfo(self.poses.getPoses())

        # l = len(self.pages)

        flag = False

        while not rospy.is_shutdown():

            if not flag:
                self.action = iter(self.getAction())
                flag = True
            else:
                try:
                    pub, pos = self.action.next()
                    pub.publish(pos)
                except StopIteration:
                    flag = False
                    self.action = []

            # if l > 1:
            #     n = r.randint(0, l-1)
            # else:
            #     n = 0
            # xposes = Poses(self.pages[n]) 
            # rospy.loginfo("**-------\nPlaying page: %s" % xposes.getTitle()) # print title
            # rospy.loginfo("**-------\nMotion: %s" % xposes.getPoses())         
            # poses = xposes.getPoses()
            # timing = xposes.getTiming()
            # for i in range(len(poses['R_ELBOW'])):
            #     for joint, pub in joints.iteritems():
            #         foo = self.mapp(poses[joint][i])
            #         pub.publish(Float64(foo))

            #     # rospy.loginfo("----------------------%s" % rospy.get_time())
            #     while rospy.get_time() - self.then < 0.3:                                         
            #         pass
            #     self.then = rospy.get_time()


            self.sleeper.sleep() 

    def getAction(self):
        chosen_action = []
        if self.page_length > 1:
            n = r.randint(0, self.page_length-1)
        else:
            n = 0
        xposes = Poses(self.pages[n]) 
        rospy.loginfo("**-------\nPlaying page: %s" % xposes.getTitle()) # print title
        rospy.loginfo("**-------\nMotion: %s" % xposes.getPoses())         
        poses = xposes.getPoses()
        timing = xposes.getTiming()
        for i in range(len(poses['R_ELBOW'])):
            for joint, pub in self.joints.iteritems():
                foo = self.mapp(poses[joint][i])
                chosen_action.append((pub, Float64(foo)))
        return chosen_action


    def mapp(self, x):
        return (x - 512.0)/512.0           

def main(args):
    rospy.init_node('jimmy_controller_node', anonymous=True)
    jimmyc = JimmyController()
    jimmyc.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)

