#!/usr/bin/env python
import sys, threading
import random as r
import xml.etree.ElementTree as etree
import decimal
from math import *
from collections import deque
import ast

from scipy.interpolate import interp1d, lagrange
import numpy as np
import matplotlib.pyplot as plt
import pywt

import rospy
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64, String, Bool

from arbotix_msgs.srv import *
from rebel_ros.srv import *

from poses1 import Poses

# arbotix_controller_path = rospkg.RosPack().get_path('arbotix_controller')

# file_interpolate = "%s/tests/interpolated_data.csv" % arbotix_controller_path
# file_timer = "%s/tests/timer_data.csv" % arbotix_controller_path
# file_pausetime = "%s/tests/pausetime_data.csv" % arbotix_controller_path

# interpolate_fh = open(file_interpolate, 'w')
# timer_fh = open(file_timer, 'w')
# pausetime_fh = open(file_pausetime, 'w')


class JimmyController(threading.Thread):
    roll = 0.0
    pan = 0.0
    then = 0

    def __init__(self):

        # List of recognized joints
        self.head_pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
        self.head_tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
        self.right_sho_pitch = rospy.Publisher('/right_sho_pitch/command', Float64, queue_size=1)
        self.right_sho_roll = rospy.Publisher('/right_sho_roll/command', Float64, queue_size=1)
        self.right_elbow = rospy.Publisher('/right_elbow/command', Float64, queue_size=1)
        self.right_elbow_twist = rospy.Publisher('/right_elbow_twist/command', Float64, queue_size=1)
        self.left_sho_pitch = rospy.Publisher('/left_sho_pitch/command', Float64, queue_size=1)
        self.left_sho_roll = rospy.Publisher('/left_sho_roll/command', Float64, queue_size=1)
        self.left_elbow = rospy.Publisher('/left_elbow/command', Float64, queue_size=1)
        self.left_elbow_twist = rospy.Publisher('/left_elbow_twist/command', Float64, queue_size=1)

        # Subscribers
        # Arbotix joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        # REBeL entry point - send a REBeL expression to this topic
        rospy.Subscriber('/gesture_command', String, self.gesture_callback)

        # Interrupt any process and quit
        rospy.Subscriber('/interrupt', Bool, self.interrupt_callback)

        # Change interpolation mode: 'kb', 'cubic', 'linear'
        rospy.Subscriber('/interpolate_mode', String, self.interpolate_mode_callback)

        # Toggle Multiresolution Filtering
        rospy.Subscriber('/multires_filtering', Bool, self.mrf_callback)

        # Toggle Idle animation
        rospy.Subscriber('/idle_mode', Bool, self.idle_callback)

        # Dynamixel/Arbotix services
        # Services to turn servos off
        self.head_pan_relax = rospy.ServiceProxy(
            '/head_pan_joint/relax', Relax)
        self.head_tilt_relax = rospy.ServiceProxy(
            '/head_tilt_joint/relax', Relax)
        self.left_elbow_relax = rospy.ServiceProxy(
            '/left_elbow/relax', Relax)
        self.left_elbow_twist_relax = rospy.ServiceProxy(
            '/left_elbow_twist/relax', Relax)
        self.left_sho_pitch_relax = rospy.ServiceProxy(
            '/left_sho_pitch/relax', Relax)
        self.left_sho_roll_relax = rospy.ServiceProxy(
            '/left_sho_roll/relax', Relax)
        self.right_elbow_relax = rospy.ServiceProxy(
            '/right_elbow/relax', Relax)
        self.right_elbow_twist_relax = rospy.ServiceProxy(
            '/right_elbow_twist/relax', Relax)
        self.right_sho_pitch_relax = rospy.ServiceProxy(
            '/right_sho_pitch/relax', Relax)
        self.right_sho_roll_relax = rospy.ServiceProxy(
            '/right_sho_roll/relax', Relax)

        # Services to turn servos on
        self.head_pan_enable = rospy.ServiceProxy(
            '/head_pan_joint/enable', Enable)
        self.head_tilt_enable = rospy.ServiceProxy(
            '/head_tilt_joint/enable', Enable)
        self.left_elbow_enable = rospy.ServiceProxy(
            '/left_elbow/enable', Enable)
        self.left_elbow_twist_enable = rospy.ServiceProxy(
            '/left_elbow_twist/enable', Enable)
        self.left_sho_pitch_enable = rospy.ServiceProxy(
            '/left_sho_pitch/enable', Enable)
        self.left_sho_roll_enable = rospy.ServiceProxy(
            '/left_sho_roll/enable', Enable)
        self.right_elbow_enable = rospy.ServiceProxy(
            '/right_elbow/enable', Enable)
        self.right_elbow_twist_enable = rospy.ServiceProxy(
            '/right_elbow_twist/enable', Enable)
        self.right_sho_pitch_enable = rospy.ServiceProxy(
            '/right_sho_pitch/enable', Enable)
        self.right_sho_roll_enable = rospy.ServiceProxy(
            '/right_sho_roll/enable', Enable)

        # Services to set speed
        # Services to turn servos on
        self.head_pan_set_speed = rospy.ServiceProxy(
            '/head_pan_joint/set_speed', SetSpeed)
        self.head_tilt_set_speed = rospy.ServiceProxy(
            '/head_tilt_joint/set_speed', SetSpeed)
        self.left_elbow_set_speed = rospy.ServiceProxy(
            '/left_elbow/set_speed', SetSpeed)
        self.left_elbow_twist_set_speed = rospy.ServiceProxy(
            '/left_elbow_twist/set_speed', SetSpeed)
        self.left_sho_pitch_set_speed = rospy.ServiceProxy(
            '/left_sho_pitch/set_speed', SetSpeed)
        self.left_sho_roll_set_speed = rospy.ServiceProxy(
            '/left_sho_roll/set_speed', SetSpeed)
        self.right_elbow_set_speed = rospy.ServiceProxy(
            '/right_elbow/set_speed', SetSpeed)
        self.right_elbow_twist_set_speed = rospy.ServiceProxy(
            '/right_elbow_twist/set_speed', SetSpeed)
        self.right_sho_pitch_set_speed = rospy.ServiceProxy(
            '/right_sho_pitch/set_speed', SetSpeed)
        self.right_sho_roll_set_speed = rospy.ServiceProxy(
            '/right_sho_roll/set_speed', SetSpeed)

        # REBeL service
        rospy.wait_for_service('rebel_parser_server')
        self.rebel_parser = rospy.ServiceProxy('rebel_parser_server', Rebel)

        # VARIABLES and helper variables
        self.joints = {'R_ELBOW': self.right_elbow,
            # 'R_ELBOW_TWIST': self.right_elbow_twist,
            'R_SHO_PITCH': self.right_sho_pitch,
            'R_SHO_ROLL': self.right_sho_roll,
            'L_ELBOW': self.left_elbow,
            # 'L_ELBOW_TWIST': self.left_elbow_twist,
            'L_SHO_PITCH': self.left_sho_pitch,
            'L_SHO_ROLL': self.left_sho_roll,
            'HEAD_PAN': self.head_pan_pub,
            'HEAD_TILT': self.head_tilt_pub}

        self.joint_pos = {'R_ELBOW': 0.0,
            'R_ELBOW_TWIST': 0.0,
            'R_SHO_PITCH': 0.0,
            'R_SHO_ROLL': 0.0,
            'L_ELBOW': 0.0,
            'L_ELBOW_TWIST': 0.0,
            'L_SHO_PITCH': 0.0,
            'L_SHO_ROLL': 0.0,
            'HEAD_PAN': 0.0,
            'HEAD_TILT': 0.0}

        self.pages = None
        self.poses = None
        self.poseTitle = "Default"
        self.init_ready = [False, False, False, False, False, False, False, False, True, True]
        self.ready = False
        self.enable_ready = False
        self.add_initial = False # False: add initial pose, True: don't add initial pose
        self.pausemode = True  # False: hard stops, True: blended in interpolated data

        self.catmull_rom_res = 50
        self.kb_res = 32

        self.after_motion = True
        self.gesture_command = False
        self.idle_time = rospy.get_time()
        self.then = rospy.get_time()

        # self.write_to_file = True
        self.loglog = True

        # Adjustable params
        self.interpolation_mode = 'kb'
        self.use_mrf = True
        self.use_idle = True

        rospack = rospkg.RosPack()

        # Load from .pagelist file
        # List of pre-made motion files in WinRME format
        # tree = etree.parse('/home/mathias/Downloads/WinRME/Chair-Poses.pagelist')
        # tree = etree.parse('%s/src/pagelists/test-poses.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/src/pagelists/Chair-Poses.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/src/pagelists/marie_curie2_edited4.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/src/pagelists/PositionSequence4.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/PositionSequence.pagelist' % rospack.get_path('rebel_ros'))

        # Uncomment these two lines to execute pre-made motions
        # self.pages = tree.findall('.//PageClass')
        # self.page_length = len(self.pages)

        self.action = []

        rospy.loginfo("Setting up JimmyController ...")

        rospy.wait_for_service('/head_pan_joint/relax')
        rospy.wait_for_service('/head_tilt_joint/relax')
        rospy.wait_for_service('/left_elbow/relax')
        rospy.wait_for_service('/left_elbow_twist/relax')
        rospy.wait_for_service('/left_sho_pitch/relax')
        rospy.wait_for_service('/left_sho_roll/relax')
        rospy.wait_for_service('/right_elbow/relax')
        rospy.wait_for_service('/right_elbow_twist/relax')
        rospy.wait_for_service('/right_sho_pitch/relax')
        rospy.wait_for_service('/right_sho_roll/relax')
        rospy.wait_for_service('/head_pan_joint/enable')
        rospy.wait_for_service('/head_tilt_joint/enable')
        rospy.wait_for_service('/left_elbow/enable')
        rospy.wait_for_service('/left_elbow_twist/enable')
        rospy.wait_for_service('/left_sho_pitch/enable')
        rospy.wait_for_service('/left_sho_roll/enable')
        rospy.wait_for_service('/right_elbow/enable')
        rospy.wait_for_service('/right_elbow_twist/enable')
        rospy.wait_for_service('/right_sho_pitch/enable')
        rospy.wait_for_service('/right_sho_roll/enable')

        self.relax_servos()
        self.enable_servos()

        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(19.75)

    def relax_servos(self):
        '''
        Turn off all servos
        '''
        self.head_pan_relax()
        self.head_tilt_relax()
        self.left_elbow_relax()
        self.left_elbow_twist_relax()
        self.left_sho_pitch_relax()
        self.left_sho_roll_relax()
        self.right_elbow_relax()
        self.right_elbow_twist_relax()
        self.right_sho_pitch_relax()
        self.right_sho_roll_relax()

    def enable_servos(self):
        '''
        Turn on all servos
        '''
        self.set_speed('slow')
        self.head_pan_enable(True)
        self.head_tilt_enable(True)
        self.left_elbow_enable(True)
        self.left_elbow_twist_enable(True)
        self.left_sho_pitch_enable(True)
        self.left_sho_roll_enable(True)
        self.right_elbow_enable(True)
        self.right_elbow_twist_enable(True)
        self.right_sho_pitch_enable(True)
        self.right_sho_roll_enable(True)
        self.enable_ready = True
        self.set_speed('max')

    def set_speed(self, s='slow'):  # 7 = slow, 20+ = fast
        speeds = {'slow': 5, 'max': 200}
        spd = speeds[s]
        self.head_pan_set_speed(spd)
        self.head_tilt_set_speed(spd)
        self.left_elbow_set_speed(spd)
        self.left_elbow_twist_set_speed(spd)
        self.left_sho_pitch_set_speed(spd)
        self.left_sho_roll_set_speed(spd)
        self.right_elbow_set_speed(spd)
        self.right_elbow_twist_set_speed(spd)
        self.right_sho_pitch_set_speed(spd)
        self.right_sho_roll_set_speed(spd)
        wait_time = min(3.0, abs(np.random.normal(loc=3.0, scale=2)))
        print("Waiting {} to set speed ...".format(wait_time))
        while rospy.get_time() - self.idle_time < wait_time:

            continue

   # CALLBACKS
    def joint_callback(self, data):
        '''
        Keep track of current joint states (positions)
        '''
        if not self.ready and self.enable_ready:
            names = data.name
            # position = [self.pospos(x) for x in data.position]
            position = data.position
            jointsxpositions = zip(names, position) # create pairs
            rospy.loginfo("data: %s" % jointsxpositions)

            for joint in jointsxpositions:
                # rospy.loginfo(joint)
                joint_name = joint[0]

                # if int(joint[1]) > 10:
                #     joint_pos = self.pospos(joint[1])
                # else:
                #     joint_pos = joint[1]

                joint_pos = joint[1]

                if joint_name == 'left_sho_roll':
                    self.joint_pos['L_SHO_ROLL'] = joint_pos
                    self.init_ready[0] = True
                    rospy.loginfo(">>>>> L_SHO_ROLL (%s)" % joint_pos)
                if joint_name == 'left_sho_pitch':
                    self.joint_pos['L_SHO_PITCH'] = joint_pos
                    self.init_ready[1] = True
                    rospy.loginfo(">>>>> L_SHO_PITCH (%s)" % joint_pos)
                if joint_name == 'left_elbow':
                    self.joint_pos['L_ELBOW'] = joint_pos
                    self.init_ready[2] = True
                    rospy.loginfo(">>>>> L_ELBOW (%s)" % joint_pos)
                if joint_name == 'right_sho_roll':
                    self.joint_pos['R_SHO_ROLL'] = joint_pos
                    self.init_ready[3] = True
                    rospy.loginfo(">>>>> R_SHO_ROLL (%s)" % joint_pos)
                if joint_name == 'right_sho_pitch':
                    self.joint_pos['R_SHO_PITCH'] = joint_pos
                    self.init_ready[4] = True
                    rospy.loginfo(">>>>> R_SHO_PITCH (%s)" % joint_pos)
                if joint_name == 'right_elbow':
                    self.joint_pos['R_ELBOW'] = joint_pos
                    self.init_ready[5] = True
                    rospy.loginfo(">>>>> R_ELBOW (%s)" % joint_pos)
                if joint_name == 'head_pan_joint':
                    self.joint_pos['HEAD_PAN'] = joint_pos
                    self.init_ready[6] = True
                    rospy.loginfo(">>>>> HEAD_PAN (%s)" % joint_pos)
                if joint_name == 'head_tilt_joint':
                    self.joint_pos['HEAD_TILT'] = joint_pos
                    self.init_ready[7] = True
                    rospy.loginfo(">>>>> HEAD_TILT (%s)" % joint_pos)
                # if joint_name == 'left_elbow_twist':
                #     self.joint_pos['L_ELBOW_TWIST'] = joint_pos
                #     self.init_ready[8] = True
                #     rospy.loginfo(">>>>> L_ELBOW_TWIST (%s)" % joint_pos)
                # if joint_name == 'right_elbow_twist':
                #     self.joint_pos['R_ELBOW_TWIST'] = joint_pos
                #     self.init_ready[9] = True
                #     rospy.loginfo(">>>>> R_ELBOW_TWIST (%s)" % joint_pos)

        self.ready = all(self.init_ready)

        # if self.ready and self.after_motion and not self.gesture_command:
        #     rospy.loginfo("OK! I'm ready")
        #     self.doIdle()

    def gesture_callback(self, gesture):
        '''
        Callback for /gesture_command topic
        gesture (str): REBeL expression to parse and execute
        '''
        # Reload the motion file if new gesture is given
        rospy.loginfo("[gesture_callback] Got new gesture: %s" % gesture.data)
        print "Greeting.data: %s" % gesture.data
        get = self.rebel_parser(gesture.data)
        word = get.word
        sequence = get.sequence
        if word:
            self.poseTitle = gesture.data
            seq = ast.literal_eval(sequence)
            rospy.loginfo("[gesture_callback] response:\n%s\n%s" % (word, seq))
            self.poses = seq
            self.ready = True
            self.after_motion = False
            self.gesture_command = True
            rospy.loginfo("OK! I'm ready to execute: {}".format(self.poseTitle))

    def idle_callback(self, gesture):
        '''
        Callback when idling
        gesture (str): REBeL expression to parse and execute
        '''
        # Reload the motion file if new gesture is given
        if self.gesture_command:
            return
        rospy.loginfo("[idle_callback] Got new gesture: %s" % gesture.data)
        print "Greeting.data: %s" % gesture.data
        get = self.rebel_parser(gesture.data)
        word = get.word
        sequence = get.sequence
        if word:
            self.poseTitle = gesture.data
            seq = ast.literal_eval(sequence)
            rospy.loginfo("[idle_callback] response:\n%s\n%s" % (word, seq))
            self.poses = seq
            self.after_motion = False
            rospy.loginfo("OK! I'm ready to execute: {}".format(self.poseTitle))

    def interrupt_callback(self, interrupt):
        rospy.loginfo("Interrupt!")
        self.ready = False
        rospy.signal_shutdown("Interrupt called!")

    def interpolate_mode_callback(self, mode):
        mode_data = mode.data
        rospy.loginfo("[interpolate_mode_callback] change to: {}".format(mode_data))
        if mode_data in ['linear', 'cubic', 'kb']:
            self.interpolation_mode = mode_data

    def mrf_callback(self, mode):
        mode_data = mode.data
        rospy.loginfo("[mrf_callback] change to: {}use mrf".format("" if mode_data else "not "))
        self.use_mrf = mode_data

    # def idle_callback(self, mode):
    #     mode_data = mode.data
    #     rospy.loginfo("[mrf_callback] change to: {}use mrf".format("" if mode_data else "not "))
    #     self.use_mrf = mode_data

    def pospos(self, x):
        # Convert joint_pos from -2.6 - 2.6 to 0 - 1020
        return min(1020, max(0, int((x + 2.6)/2.6 * 512)))

    def resetPose(self):
        rospy.loginfo("***ResetPoses***")
        self.set_speed('slow')
        wait_time = abs(np.random.normal(loc=2.0, scale=1))
        while rospy.get_time() - self.idle_time < wait_time:
            continue
        for joint, pub in self.joints.items():
            # pub.publish(Float64(self.joint_pos[joint]))
            pub.publish(Float64(0.0))

        self.left_elbow_twist.publish(Float64(0.0))
        self.right_elbow_twist.publish(Float64(0.0))
        self.set_speed('max')

    def doIdle(self):
        # pow = rospy.get_time()
        # for joint, pub in self.joints.iteritems():
        #     rospy.loginfo("Looping ... ")
        #     g = np.random.choice([-1, 1]) * 0.05
        #     if 'R_' in joint:
        #         gpos = 0.1 * np.sin(1.2*pow) + 0.1 * np.cos(1.2*pow + 0.21)
        #     elif 'L_' in joint:
        #         gpos = 0.1 * np.cos(1.2*pow) + 0.1 * np.sin(1.2*pow + 0.2)
        #     elif '_PAN' in joint:
        #         if rospy.get_time() - pow > np.random.normal(loc=5.0, scale=2.0):
        #             gpos = np.sin(0.05 * pow)
        #     # else:
        #     #     gpos = 0.05*np.cos(pow)

        #     if self.after_motion:
        #         gpos = self.joint_pos[joint] + gpos
        #     pub.publish(Float64(gpos))  # Return to home
        #     self.joint_pos[joint] = gpos
        # else:
        #     self.after_motion = False
        #     self.ready = True
        #     if rospy.get_time() - self.idle_time > 3.0 and abs(np.random.randn()) > 1.1:
        #         wait = String()
        #         wait.data = 'waiting'
        #         self.gesture_callback(wait)
        # self.after_motion = False
        # self.ready = True
        wait_time = abs(np.random.normal(loc=3.0, scale=2))
        while rospy.get_time() - self.idle_time < wait_time:
            rospy.loginfo("Listening for commands...")
            pow = rospy.get_time()
            for joint, pub in self.joints.iteritems():
                rospy.loginfo("Waiting ... {}".format(joint))
                g = np.random.choice([-1, 1]) * 0.05
                if 'R_' in joint:
                    gpos = 0.05 * (np.sin(1.2*pow) + np.cos(1.2*pow + 0.21))
                elif 'L_' in joint:
                    gpos = -0.05 * (np.cos(1.2*pow) + np.sin(1.2*pow + 0.2))
                elif '_PAN' in joint:
                    if rospy.get_time() - pow > np.random.normal(loc=5.0, scale=2.0):
                        gpos = 0.1* np.sin(0.05 * pow)
                # else:
                #     gpos = 0.05*np.cos(pow)

                pub.publish(Float64(gpos))  # Return to home

        else:
            rospy.loginfo("Boring ...")
            wait = String()
            action = np.random.choice(['waiting', 'wave2','muscle_flex'])
            # wait.data = 'waiting'
            # self.idle_callback(wait)
            self.idle_time = rospy.get_time()

        # self.left_elbow_twist.publish(Float64(0.0))
        # self.right_elbow_twist.publish(Float64(0.0))

    def run(self):

        self.resetPose()

        if self.pages:
            l = len(self.pages)
        else:
            l = 0

        flag = False
        n = 0

        new_poses = {}
        posex_length = 0


        while not rospy.is_shutdown():
            if self.ready and self.after_motion and not self.gesture_command:
                rospy.loginfo("OK! I'm ready")
                self.doIdle()
                continue

            if self.ready and not self.after_motion:

                # if l > 1:
                #     n = r.randint(0, l-1)
                # else:
                #     n = 0
                # xposes = Poses(self.pages[n])
                # rospy.loginfo("**-------\nPlaying page: %s" % xposes.getTitle()) # print title
                # rospy.loginfo("**-------\nMotion: %s" % xposes.getPoses())
                xposes = Poses()

                # if not self.pages and not self.poses:
                #     continue
                if self.pages:  # From pagelist
                    xposes.loadPage(self.pages[n])
                elif self.poses:  # From REBeL
                    xposes.setTitle(self.poseTitle)
                    xposes.loadPoses(self.poses)
                else:
                    continue

                # xposes.loadPage(self.pages[n])    # for random
                # xposes.setPoses(self.poses)

                # Only add initial pose at the very beginning - when the node first starts
                if self.add_initial:
                    rospy.loginfo("INITIAL JOINT POSE: %s" % self.joint_pos)
                    xposes.addInitialPose(self.joint_pos)
                    # self.add_initial = True
                poses = xposes.getPoses()   # Load poses
                timing = xposes.getTiming()

                rospy.loginfo("Going to do %s!!!" % xposes.getTitle())

                # if self.write_to_file:
                #     try:
                #         interpolate_fh = open(file_interpolate, 'w')
                #         timer_fh = open(file_timer, 'w')
                #         if interpolate_fh == None:
                #             raise IOError
                #         if timer_fh == None:
                #             raise IOError
                #     except IOError as e:
                #         print "Problems opening files!!! %s, %s\nCause: %s" % (file_interpolate, file_timer, e)



                for joint, pub in self.joints.iteritems():
                    rospy.loginfo("Joint: %s" % joint)
                    _posex = poses[joint]
                    _timer = timing['Time']
                    rospy.loginfo("len(_posex): %s vs len(timing): %s" % (len(_posex), len(timing['PauseTime'])))

                    # if self.write_to_file:
                    #     if not interpolate_fh.closed:
                    #         interpolate_fh.write("JOINT: %s\n\n" % joint)
                    #         interpolate_fh.write("Original motion:\n")
                    #         interpolate_fh.write("%s" % _posex)

                    #     if not timer_fh.closed:
                    #         timer_fh.write("JOINT: %s\n\n" % joint)
                    #         timer_fh.write("Original timing:\n")
                    #         timer_fh.write("%s" % _timer)

                    if self.loglog:
                        rospy.loginfo("LOGLOG: JOINT: %s" % joint)
                        rospy.loginfo("LOGLOG: Original motion: %s" % _posex)
                        rospy.loginfo("LOGLOG: Original timing: %s" % _timer)


                    # If self.pausemode true: embed frames into motion data
                    # Version 1.0
                    # if self.pausemode:
                    #     posex = []
                    #     timer = []
                    #     div = 100.0
                    #     pauses = timing['PauseTime']
                    #     for m in range(len(timing['PauseTime'])):
                    #         # The number of frames to embed = PauseTime/100
                    #         mult = int((round(pauses[m]/div)-1))
                    #         posex += [[_posex[m]] + [_posex[m]]*mult]
                    #         timer += [[_timer[m]] + [_timer[m]]*mult]
                    #     posex = [f for s in posex for f in s]
                    #     timer = [t for j in timer for t in j]
                    #     timing['Timer'] = timer
                    # else:
                    #     posex = list(_posex)
                    #     timing['Timer'] = list(timing['Time'])

                    # # Version 1.5
                    if self.pausemode:
                        frame_constant = 8.0
                        if self.interpolation_mode == 'kb':
                            frame_constant = 4.0
                        posex = []
                        timer = []
                        pauses = timing['PauseTime']
                        for m in range(len(timing['PauseTime'])):
                            # The number of frames to embed = PauseTime/100
                            # steps = _timer[m] * 8.0/50
                            mult = 0
                            try:
                                steps = max(5, (_timer[m] % 50) * frame_constant/50)
                                # mult = int((round(pauses[m]/100.0)-1))
                                # mult = int(round(pauses[m] / (3 * steps)))
                                mult = max(int(round(pauses[m] / (3 * steps)))//2, 1)
                            except ZeroDivisionError as zde:
                                rospy.logerr("{}: steps: {}, _timer[{}]: {}, mult: {}".format(zde, steps, m, _timer[m], mult))
                            posex += [[_posex[m]] + [_posex[m]]*mult]
                            timer += [[_timer[m]] + [_timer[m]]*mult]
                        posex = [f for s in posex for f in s]
                        timer = [t for j in timer for t in j]
                        timing['Timer'] = timer
                    else:
                        posex = list(_posex)
                        timing['Timer'] = list(timing['Time'])

                    # Version 2.0
                    # if self.pausemode:
                    #     posex = []
                    #     timer = []
                    #     pauses = timing['PauseTime']
                    #     for m in range(len(timing['PauseTime'])):
                    #         # The number of frames to embed = PauseTime/100
                    #         # mult = int((round(pauses[m]/100.0)-1))
                    #         posex += [[_posex[m]] + [_posex[m]]]
                    #         pause = timing['PauseTime'][m] * 10 # the actual time to pause in milliseconds
                    #         steps = pause/20.0    # the number of interpolatoin points (20ms = time to transmit each data)
                    #         timeval = steps*50/8.0
                    #         timer += [[_timer[m]] + [timeval]]
                    #     posex = [f for s in posex for f in s]
                    #     timer = [t for j in timer for t in j]
                    #     timing['Timer'] = timer
                    # else:
                    #     posex = list(_posex)
                    #     timing['Timer'] = list(timing['Time'])


                    # Choose interpolation method: linear, cubic, or lagrange
                    # So far lagrange has a lot of issues (movements too big)

                    fposex = self.chooseInterpolate(posex, timing, self.interpolation_mode)
                    # rospy.loginfo("lagrange fPosex[%s]: %s" % (joint, fposex))

                    # if self.write_to_file:
                    #     interpolate_fh.write("Interpolation mode: %s\n" % self.interpolation_mode)
                    #     interpolate_fh.write("Pausemode: %s\n" % self.pausemode)
                    #     if self.pausemode:
                    #         interpolate_fh.write("Pa100usemode adjusted data:\n%s" % posex)
                    #     interpolate_fh.write("Interpolated data:\n%s" % fposex)
                    #     interpolate_fh.write("\n-----\n")

                    #     timer_fh.write("Interpolation mode: %s\n" % self.interpolation_mode)
                    #     timer_fh.write("Pausemode: %s\n" % self.pausemode)
                    #     timer_fh.write("Adjusted timing:\n%s" % timing['Timer'])

                    if self.loglog:
                        rospy.loginfo("LOGLOG: Interpolation mode: %s" % self.interpolation_mode)
                        rospy.loginfo("LOGLOG: Pausemode: %s" % self.pausemode)
                        if self.pausemode:
                            rospy.loginfo("LOGLOG: Pausemode adjusted data: %s" % posex)
                            rospy.loginfo("LOGLOG: Pausemode adjusted timer: %s" % timing['Timer'])
                        else:
                            rospy.loginfo("LOGLOG: data: %s" % posex)
                            rospy.loginfo("LOGLOG: timer: %s" % timing['Timer'])
                        rospy.loginfo("LOGLOG: Interpolated data: %s" % fposex)
                        rospy.loginfo("LOGLOG: Interpolated data (abtx): %s" % [self.pospos(x) for x in fposex])

                    # new_poses is the final interpolated data
                    new_poses[joint] = fposex
                    posex_length = len(fposex)

                p = len(timing['Timer'])
                # p = len(timing['PauseTime'])
                if not self.pausemode:
                    pause = [t * 0.01 for t in timing['PauseTime']]

                if self.loglog and not self.pausemode:
                    rospy.loginfo("LOGLOG: PauseTime data: %s" % pause)
                print("\n****NEW POSES****\n{}".format(new_poses))
                # Execute all motion until finished
                for i in range(posex_length-1):
                    mx = posex_length/max(1, p + 1)

                    # Uncomment to move step by step
                    # if (i % mx == 0):
                    #     ind = i/mx
                    #     rospy.loginfo("LOGLOLG: PauseTime: %s" % pauses[ind])
                    #     raw_input("Press Enter to step through...")

                    for joint, pub in self.joints.iteritems():
                        pos = new_poses[joint][i]
                        pub.publish(Float64(pos))

                    d = 0.02

                    if (i % mx == 0) and not self.pausemode:
                        ind = i/mx
                        d += pause[ind]
                        rospy.loginfo("Wait for: %s (%s) " % (d, pause[ind]*100))

                        if self.loglog:
                            rospy.loginfo("LOGLOG: PauseTime: %s/%s" % (d, pause[ind]*100))

                    self.then = rospy.get_time()
                    while rospy.get_time() - self.then < d:
                        pass
                    self.then = rospy.get_time()

                    # Quit/immediately stop if interrupted
                    if not self.ready:
                        rospy.signal_shutdown('Interrupted')

                self.resetPose()
                # self.relax_servos()
                then = rospy.get_time()
                while rospy.get_time() - then < 1.5:
                    pass
                self.enable_servos()

                self.after_motion = True
                if self.gesture_command:
                    self.gesture_command = False

            # if self.write_to_file:
            #     try:
            #         if not interpolate_fh.closed:
            #             interpolate_fh.close()
            #         if not timer_fh.closed:
            #             timer_fh.close()
            #         if not self.pausemode:
            #             if not pausetime_fh.closed:
            #                 pausetime_fh.close()
            #     except IOError:
            #         rospy.loginfo("Having problems closing files!")
            #         print "Having problems closing files!"

            else:
                rospy.loginfo("Is Not ready ...")
                # if self.after_motion:
                #     self.doIdle()

            posex_length = 0
            # self.ready = False
            self.idle_time = rospy.get_time()  # Reset idle timer
            self.pages = None
            self.poses = None
            self.sleeper.sleep()
            # rospy.signal_shutdown("Only running once.")
            # return 0

    def chooseInterpolate(self, posex, timing, interp='cubic'):
        if interp in ['cubic','lagrange','linear','kb']:
            return self.interpolate2(posex, timing, interp, mrf=self.use_mrf)
        # elif interp == 'linear':
        #     return self.interpolate(posex, timing)
        elif interp == None:
            return None
        else:
            raise ValueError("Invalid interpolation type")
            rospy.signal_shutdown('Goodbye!')


    def getAction(self):
        chosen_action = []
        fpose = []
        if self.page_length > 1:
            n = r.randint(0, self.page_length-1)
        else:
            n = 0
        xposes = Poses(self.pages[n])
        rospy.loginfo("**-------\nPlaying page: %s" % xposes.getTitle()) # print title
        rospy.loginfo("**-------\nMotion: %s" % xposes.getPoses())
        poses = xposes.getPoses()
        timing = xposes.getTiming()

        poses_ = self.interpolate(poses, timing)

        for i in range(len(poses_['R_ELBOW'])):
            for joint, pub in self.joints.iteritems():
                foo = self.mapp(poses_[joint][i])
                fpose.append((pub, Float64(foo)))
            chosen_action.append(fpose)
        # rospy.loginfo(chosen_action)

        return chosen_action

    def interpolate(self, fubar, timing):
        fb = []
        timer = timing['Timer']
        for i in range(len(fubar)-1):
            f1 = fubar[i]
            f2 = fubar[i+1]
            diff = f2 - f1
            steps = int(timer[i+1] * 8/50.0)
            rospy.loginfo("Duration/steps: %s/%s" % (timer[i], steps))
            increment = float(diff)/float(steps)
            print increment
            fx = []
            for j in range(steps+1):
                fx += [self.mapp(f1 + j*increment)]

            print fx
            fb += fx
        print "%s (%s)" % (fb, len(fb))
        return fb

    def interpolate2(self, fubar, timing, interp, mrf=True):
        l = len(fubar)
        x = np.linspace(0, l-1, num=l, endpoint=True)
        y = fubar
        # f = interp1d(x,y)

        steps = 0
        for t in timing['Timer']:
            steps += int(t * 8/50.0)

        xnew = np.linspace(0, l-1, num=steps, endpoint=True)
        rospy.loginfo("Interpolate2 steps: %s" % steps)

        if steps == 0:
            rospy.logwarn("[interpolate2]: steps: {}".format(steps))
        if interp == 'cubic' or interp == 'linear':
            try:
                f2 = interp1d(x,y, kind=interp)
            except ValueError as ve:
                rospy.logerr("[interpolate2]: {}".format(ve))
                return []

        elif interp == 'lagrange':
            f2 = lagrange(x, y)

        elif interp == 'catmull-rom':
            x_intpol, y_intpol = self.catmull_rom(x, y, steps)
            # plt.figure()
            # plt.scatter(x, y)
            # plt.plot(x_intpol, y_intpol)
            # plt.show()
            return [self.mapp(f) for f in y_intpol]

        elif interp == 'kb':
            if len(x) <= 0 or len(y) <= 0:
                return fubar
            x_intpol, y_intpol = self.kbinterp(x, y, self.kb_res)  # kbres=32 seems to work well
            # plt.figure()
            # plt.scatter(x, y)
            # plt.plot(x_intpol, y_intpol)
            # plt.show()
            result = [self.mapp(f) for f in y_intpol]
            result += [0.0]
            if mrf:  # Apply MRF if flagged
                return self.wavedef(result)
            return result

        else:
            raise ValueError('Invalid interpolation type!')
            rospy.signal_shutdown('Goodbye!')
        # steps = 0
        # for t in timing['Timer']:
        #     steps += int(t * 8/50.0)

        # xnew = np.linspace(0, l-1, num=steps, endpoint=True)

        fx = [self.mapp(f) for f in f2(xnew)]
        fx = self.wavedef(fx)
        return fx

        # Plotting
        # axes = plt.gca()
        # axes.set_ylim([1200, 3000])
        # # plt.plot(x,y,'o', xnew, f(xnew), '-', xnew, f2(xnew), '--', xnew, f3(xnew), 'x')
        # # plt.plot(x,y,'o', xnew, f(xnew), '-', xnew, f2(xnew), '--', xnew, f3(xnew), '+', xnew, f4(xnew), 's')
        # data, = plt.plot(x,y,'o', label='data')
        # linear, = plt.plot(xnew, f(xnew), '-', label='linear')
        # cubic, = plt.plot(xnew, f2(xnew), '--', label='cubic')
        # lagrang, = plt.plot(xnew, f3(xnew), 'g^', label='lagrange')
        # plt.legend([data, linear, cubic, lagrang])
        # plt.show()

    def mapp(self, x):
        decimal.getcontext().prec=7
        return (x - 512.0)/512.0 * 2.6

    # Ref: https://github.com/vmichals/python-algos/blob/master/catmull_rom_spline.py
    # def catmull_rom_one_point(self, x, v0, v1, v2, v3):
    #     """Computes interpolated y-coord for given x-coord using Catmull-Rom.

    #     Computes an interpolated y-coordinate for the given x-coordinate between
    #     the support points v1 and v2. The neighboring support points v0 and v3 are
    #     used by Catmull-Rom to ensure a smooth transition between the spline
    #     segments.
    #     Args:
    #         x: the x-coord, for which the y-coord is needed
    #         v0: 1st support point
    #         v1: 2nd support point
    #         v2: 3rd support point
    #         v3: 4th support point
    #     """
    #     c1 = 1. * v1
    #     c2 = -.5 * v0 + .5 * v2
    #     c3 = 1. * v0 + -2.5 * v1 + 2. * v2 -.5 * v3
    #     c4 = -.5 * v0 + 1.5 * v1 + -1.5 * v2 + .5 * v3
    #     return (((c4 * x + c3) * x + c2) * x + c1)

    # def catmull_rom(self, p_x, p_y, res):
    #     """Computes Catmull-Rom Spline for given support points and resolution.

    #     Args:
    #         p_x: array of x-coords
    #         p_y: array of y-coords
    #         res: resolution of a segment (including the start point, but not the
    #             endpoint of the segment)
    #     """
    #     res = int(ceil(self.catmull_rom_res * res))
    #     # res = self.catmull_rom_res
    #     # create arrays for spline points
    #     x_intpol = np.empty(res*(len(p_x)-1) + 1)
    #     y_intpol = np.empty(res*(len(p_x)-1) + 1)

    #     # set the last x- and y-coord, the others will be set in the loop
    #     x_intpol[-1] = p_x[-1]
    #     y_intpol[-1] = p_y[-1]

    #     # loop over segments (we have n-1 segments for n points)
    #     for i in range(len(p_x)-1):
    #         # set x-coords
    #         x_intpol[i*res:(i+1)*res] = np.linspace(
    #             p_x[i], p_x[i+1], res, endpoint=False)
    #         if i == 0:
    #             # need to estimate an additional support point before the first
    #             y_intpol[:res] = np.array([
    #                 self.catmull_rom_one_point(
    #                     x,
    #                     p_y[0] - (p_y[1] - p_y[0]), # estimated start point,
    #                     p_y[0],
    #                     p_y[1],
    #                     p_y[2])
    #                 for x in np.linspace(0.,1.,res, endpoint=False)])
    #         elif i == len(p_x) - 2:
    #             # need to estimate an additional support point after the last
    #             y_intpol[i*res:-1] = np.array([
    #                 self.catmull_rom_one_point(
    #                     x,
    #                     p_y[i-1],
    #                     p_y[i],
    #                     p_y[i+1],
    #                     p_y[i+1] + (p_y[i+1] - p_y[i]) # estimated end point
    #                 ) for x in np.linspace(0.,1.,res, endpoint=False)])
    #         else:
    #             y_intpol[i*res:(i+1)*res] = np.array([
    #                 self.catmull_rom_one_point(
    #                     x,
    #                     p_y[i-1],
    #                     p_y[i],
    #                     p_y[i+1],
    #                     p_y[i+2]) for x in np.linspace(0.,1.,res, endpoint=False)])

    #     return (x_intpol, y_intpol)

    def kbinterp(self, p_x, p_y, res, t=[0.0], c=[0.0], b=[0.0]):
        # res = int(ceil(self.kb_res * res))

        # create arrays for spline points
        x_intpol = np.empty(res*(len(p_x)-1) + 1)
        y_intpol = np.empty(res*(len(p_x)-1) + 1)

        # set the last x- and y-coord, the others will be set in the loop
        # x_intpol[0] = p_x[0]
        # y_intpol[0] = p_y[0]
        # x_intpol[-1] = p_x[-1]
        # y_intpol[-1] = p_y[-1]

        # Check tension, continuity, bias values; if only one, broadcast
        # else, must have the same number as data points
        if len(t) != len(p_x):
            tension = np.full((len(p_x),), t[0])  # Fill with the first value of t
        else:
            tension = t

        if len(c) != len(p_x):
            continuity = np.full((len(p_x),), c[0])
        else:
            continuity = c

        if len(b) != len(p_x):
            bias = np.full((len(p_x),), b[0])
        else:
            bias = b

        # loop over segments (we have n-1 segments for n points)
        for i in range(len(p_x)-1):
            # set x-coords
            x_intpol[i*res:(i+1)*res] = np.linspace(
                p_x[i], p_x[i+1], res, endpoint=False)
            if i == 0:
                # need to estimate an additional support point before the first
                y_intpol[:res] = np.array([
                    self.kb_one_point(
                        x,
                        p_y[0] - (p_y[1] - p_y[0]), # estimated start point,
                        p_y[0],
                        p_y[1],
                        p_y[2],
                        tension[0],
                        continuity[0],
                        bias[0])
                    for x in np.linspace(0.,1.,res, endpoint=False)])
            elif i == len(p_x) - 2:
                # need to estimate an additional support point after the last
                y_intpol[i*res:-1] = np.array([
                    self.kb_one_point(
                        x,
                        p_y[i-1],
                        p_y[i],
                        p_y[i+1],
                        p_y[i+1] + (p_y[i+1] - p_y[i]),
                        tension[-1],
                        continuity[-1],
                        bias[-1] # estimated end point
                    ) for x in np.linspace(0.,1.,res, endpoint=False)])
            else:
                y_intpol[i*res:(i+1)*res] = np.array([
                    self.kb_one_point(
                        x,
                        p_y[i-1],
                        p_y[i],
                        p_y[i+1],
                        p_y[i+2],
                        tension[i],
                        continuity[i],
                        bias[i]) for x in np.linspace(0.,1.,res, endpoint=False)])

        # :-10 is HACK to remove some weird values at the end (cause unknown)
        return (x_intpol[:-10], y_intpol[:-10])

    def kb_one_point(self, x, v0, v1, v2, v3, t, c, b):
        """Computes interpolated y-coord for given x-coord for Kochanek Bartels.

        Computes an interpolated y-coordinate for the given x-coordinate between
        the support points v1 and v2. The neighboring support points v0 and v3 are
        used by Catmull-Rom to ensure a smooth transition between the spline
        segments.
        Args:
            x: the x-coord, for which the y-coord is needed
            v0: 1st support point
            v1: 2nd support point
            v2: 3rd support point
            v3: 4th support point
        """
        TSi = 0.5*((1-t)*(1-c)*(1+b)*(v2 - v1) + (1-t)*(1+c)*(1-b)*(v3 - v2))
        TDi = 0.5*((1-t)*(1+c)*(1+b)*(v1 - v0) + (1-t)*(1-c)*(1-b)*(v2 - v1))
        C = np.array([v1, v2, TDi, TSi])
        S = np.array([x**3, x**2, x, 1])
        H = np.matrix([[2, -2, 1, 1], [-3, 3, -2, -1], [0, 0, 1, 0], [1, 0, 0, 0]])
        result = C * (S * H).T
        return result[0,0]

    def wavedef(self, data, kernel='db5', mode='simple'):

        kern = pywt.Wavelet(kernel)
        coeffs = pywt.wavedec(data, kern)

        # depth
        k = len(coeffs)
        coeffs_to_save = k//2

        # modifier
        alpha = {0: 1.0, 1: 1.0, 2: 1.0, 5: 1.5}

        # Only keep the first half - turn the bottom half to zero
        for j in range(k):
            if j == coeffs_to_save:
                coeffs[j] = abs(np.random.normal(loc=0.0, scale=0.5)) * coeffs[j]
            if j > coeffs_to_save + 1:
                coeffs[j] = np.multiply(1.2, coeffs[j])
            if j in alpha.keys():
                coeffs[j] *= alpha[j]
            else:
                coeffs[j] *= 1.2

        reconstruct = pywt.waverec(coeffs, kern)
        return reconstruct

def main(args):
    rospy.init_node('jimmy_controller_node_v2', anonymous=True)
    jimmyc = JimmyController()
    jimmyc.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)
