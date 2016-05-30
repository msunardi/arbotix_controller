#!/usr/bin/env python
import sys, threading
import rospy
from sensor_msgs.msg import JointState
from collections import deque

from arbotix_msgs.srv import *

from poses import Poses

class JimmyTrainer(threading.Thread):

    def __init__(self):
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        self.head_pan_relax = rospy.ServiceProxy(
            '/head_pan_joint/relax', Relax)
        self.head_tilt_relax = rospy.ServiceProxy(
            '/head_tilt_joint/relax', Relax)
        self.left_elbow_relax = rospy.ServiceProxy(
            '/left_elbow/relax', Relax)
        self.left_sho_pitch_relax = rospy.ServiceProxy(
            '/left_sho_pitch/relax', Relax)
        self.left_sho_roll_relax = rospy.ServiceProxy(
            '/left_sho_roll/relax', Relax)
        self.right_elbow_relax = rospy.ServiceProxy(
            '/right_elbow/relax', Relax)        
        self.right_sho_pitch_relax = rospy.ServiceProxy(
            '/right_sho_pitch/relax', Relax)        
        self.right_sho_roll_relax = rospy.ServiceProxy(
            '/right_sho_roll/relax', Relax)

        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(10)

    def joint_callback(self, data):
    	names = data.name
    	position = data.position
    	fubar = zip(names, position) # create pairs
    	rospy.loginfo("data: %s" % fubar)
    	# rospy.loginfo("Positions: %s" % position)

    def run(self):
    	rospy.wait_for_service('/head_pan_joint/relax')
        rospy.wait_for_service('/head_tilt_joint/relax')
        rospy.wait_for_service('/left_elbow/relax')
        rospy.wait_for_service('/left_sho_pitch/relax')
        rospy.wait_for_service('/left_sho_roll/relax')
        rospy.wait_for_service('/right_elbow/relax')
    	rospy.wait_for_service('/right_sho_pitch/relax')
    	rospy.wait_for_service('/right_sho_roll/relax')

    	self.relax_servos()

    	while not rospy.is_shutdown():
    		self.sleeper.sleep()

    def relax_servos(self):
    	self.head_pan_relax()
    	self.head_tilt_relax()
    	self.left_elbow_relax()
    	self.left_sho_pitch_relax()
    	self.left_sho_roll_relax()
    	self.right_elbow_relax()
    	self.right_sho_pitch_relax()
    	self.right_sho_roll_relax()

def main(args):
    rospy.init_node('jimmy_trainer_node', anonymous=True)
    jimmyt = JimmyTrainer()
    jimmyt.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)   