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

import rospy
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64, String, Bool

from arbotix_msgs.srv import *
from rebel_ros.srv import *
from midi_motion.srv import *

from poses import Poses

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
        self.head_pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=10)        
        self.head_tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=10)
        self.right_sho_pitch = rospy.Publisher('/right_sho_pitch/command', Float64, queue_size=10)
        self.right_sho_roll = rospy.Publisher('/right_sho_roll/command', Float64, queue_size=10)
        self.right_elbow = rospy.Publisher('/right_elbow/command', Float64, queue_size=10)
        self.left_sho_pitch = rospy.Publisher('/left_sho_pitch/command', Float64, queue_size=10)
        self.left_sho_roll = rospy.Publisher('/left_sho_roll/command', Float64, queue_size=10)
        self.left_elbow = rospy.Publisher('/left_elbow/command', Float64, queue_size=10)

        # self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        # self.gesture_command = rospy.Subscriber('/gesture_command', String, self.gesture_callback)
        # self.interrupt = rospy.Subscriber('/interrupt', Bool, self.interrupt_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        rospy.Subscriber('/gesture_command', String, self.gesture_callback)
        rospy.Subscriber('/interrupt', Bool, self.interrupt_callback)

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

        self.head_pan_enable = rospy.ServiceProxy(
            '/head_pan_joint/enable', Enable)
        self.head_tilt_enable = rospy.ServiceProxy(
            '/head_tilt_joint/enable', Enable)
        self.left_elbow_enable = rospy.ServiceProxy(
            '/left_elbow/enable', Enable)
        self.left_sho_pitch_enable = rospy.ServiceProxy(
            '/left_sho_pitch/enable', Enable)
        self.left_sho_roll_enable = rospy.ServiceProxy(
            '/left_sho_roll/enable', Enable)
        self.right_elbow_enable = rospy.ServiceProxy(
            '/right_elbow/enable', Enable)        
        self.right_sho_pitch_enable = rospy.ServiceProxy(
            '/right_sho_pitch/enable', Enable)        
        self.right_sho_roll_enable = rospy.ServiceProxy(
            '/right_sho_roll/enable', Enable)

        # Wait for REBeL Service
        rospy.wait_for_service('rebel_parser_server')
        self.rebel_parser = rospy.ServiceProxy('rebel_parser_server', Rebel)

        # Wait for midi_motion Service
        rospy.wait_for_service('midi_motion_server')
        self.midi_motion = rospy.ServiceProxy('midi_motion_server', MidiInfo)

        self.joints = {'R_ELBOW': self.right_elbow,
            'R_SHO_PITCH': self.right_sho_pitch,
            'R_SHO_ROLL': self.right_sho_roll,
            'L_ELBOW': self.left_elbow,
            'L_SHO_PITCH': self.left_sho_pitch,
            'L_SHO_ROLL': self.left_sho_roll,
            'HEAD_PAN': self.head_pan_pub,
            'HEAD_TILT': self.head_tilt_pub}

        self.joint_pos = {'R_ELBOW': 512,
            'R_SHO_PITCH': 512,
            'R_SHO_ROLL': 512,
            'L_ELBOW': 512,
            'L_SHO_PITCH': 512,
            'L_SHO_ROLL': 512,
            'HEAD_PAN': 512,
            'HEAD_TILT': 512}

        self.pages = None
        self.poses = None
        self.poseTitle = "Default"
        self.init_ready = [False]*8
        self.ready = False
        self.enable_ready = False
        self.add_initial = False # False: add initial pose, True: don't add initial pose        
        self.pausemode = True  # False: hard stops, True: blended in interpolated data
        self.interpolation_mode = 'cubic'
        self.interpolation_mode = 'catmull-rom'
        self.use_midi = True
        self.midi_title = "Default"
        self.catmull_rom_res = 0.04

        # self.write_to_file = True
        self.loglog = True

        # self.interpolate_fh = None
        # self.timer_fh = None
        # self.pausetime_fh = None

        self.rospack = rospkg.RosPack()

        # Load from .pagelist file
        # tree = etree.parse('/home/mathias/Downloads/WinRME/Chair-Poses.pagelist')
        # tree = etree.parse('%s/src/pagelists/test-poses.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/src/pagelists/Chair-Poses.pagelist' %( rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/src/pagelists/marie_curie2_edited4.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/src/pagelists/PositionSequence4.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/PositionSequence.pagelist' % rospack.get_path('rebel_ros'))
        # self.pages = tree.findall('.//PageClass')
        # self.page_length = len(self.pages)       

        self.action = []    

        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(50)

    def relax_servos(self):
        self.head_pan_relax()
        self.head_tilt_relax()
        self.left_elbow_relax()
        self.left_sho_pitch_relax()
        self.left_sho_roll_relax()
        self.right_elbow_relax()
        self.right_sho_pitch_relax()
        self.right_sho_roll_relax()

    def enable_servos(self):
        self.head_pan_enable(True)
        self.head_tilt_enable(True)
        self.left_elbow_enable(True)
        self.left_sho_pitch_enable(True)
        self.left_sho_roll_enable(True)
        self.right_elbow_enable(True)
        self.right_sho_pitch_enable(True)
        self.right_sho_roll_enable(True)
        self.enable_ready = True

    def joint_callback(self, data):        
        # rospy.loginfo("Positions: %s" % position)
        # self.ready = False
        # self.enable_ready = True
        if not self.ready and self.enable_ready:
            names = data.name
            # position = [self.pospos(x) for x in data.position]
            position = data.position
            fubar = zip(names, position) # create pairs            
            rospy.loginfo("data: %s" % fubar)

            for joint in fubar:
                # rospy.loginfo(joint)
                joint_name = joint[0]
                joint_pos = self.pospos(joint[1])
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
        if not False in self.init_ready:
            # rospy.loginfo("OK! I'm ready")
            self.ready = True

    def gesture_callback(self, gesture):
        caller = "[gesture_callback]"
        # Reload the motion file if new gesture is given
        rospy.loginfo("[gesture_callback] Got new gesture: %s" % gesture.data)
        print "Greeting.data: %s" % gesture.data
        get = self.rebel_parser(gesture.data)
        word = get.word
        sequence = get.sequence

        if self.use_midi:
            # self.midi_title = r.choice(['lullaby_of_birdland', 'dancing_queen', 'pink_panther2', 'auld_lang_syne'])
            # med_file_path = '%s/src/%s.med' % (self.rospack.get_path('midi_motion'), self.midi_title)
            # med_file_path = '%s/src/%s.med' % (self.rospack.get_path('midi_motion'), 'lullaby_of_birdland')
            # med_file_path = '%s/src/%s.med' % (self.rospack.get_path('midi_motion'), 'dancing_queen')
            # med_file_path = '%s/src/%s.med' % (self.rospack.get_path('midi_motion'), 'pink_panther')
            med_file_path = '%s/src/%s.med' % (self.rospack.get_path('midi_motion'), 'pink_panther2')
            # med_file_path = '%s/src/%s.med' % (self.rospack.get_path('midi_motion'), 'auld_lang_syne')
            rospy.loginfo("%s med_file_path: %s" % (caller, med_file_path))
            mididata = self.midi_motion(med_file_path)
        
        if word:
            self.poseTitle = gesture.data          
            seq = ast.literal_eval(sequence)
            if self.use_midi:
                rospy.loginfo("%s Applying MIDI data from: ... %s" % (caller, self.midi_title))
                rospy.loginfo("%s original timing 'Time': %s" % (caller, seq['Time']))
                seq['PauseTime'] = self.update_timing_data(seq['PauseTime'], ast.literal_eval(mididata.timing))
                seq['Time'] = self.update_timing_data(seq['Time'], ast.literal_eval(mididata.timing))
                rospy.loginfo("%s updated timing 'Time': %s" % (caller, seq['Time']))
            rospy.loginfo("%s response:\n%s\n%s" % (caller, word, seq))
            
            self.poses = seq
            self.ready = True
        # tree = etree.parse('%s/PositionSequence.pagelist' % rospack.get_path('rebel_ros'))
        # self.pages = tree.findall('.//PageClass')
        # self.page_length = len(self.pages)
        # self.ready = True

    def update_timing_data(self, sequence, midi_timing):
        print "Sequence: %s" % sequence
        rospy.loginfo("[update_timing_data]: %s" % midi_timing)
        l = len(sequence)
        timing_delta = [midi_timing[i+1]-midi_timing[i] for i in range(len(midi_timing)-1)]
        if len(timing_delta) >= l:
            return [t*200 for t in timing_delta[:l]] # Only return timing data as many as sequence data
        else:
            return sequence['Time']


    def interrupt_callback(self, interrupt):
        rospy.loginfo("Interrupt!")
        self.ready = False

    def pospos(self, x):
        # Convert joint_pos from -2.6 - 2.6 to 0 - 1020
        return min(1020, max(0, int((x + 2.6)/2.6 * 512)))
        
    def run(self):
        rospy.loginfo("Setting up JimmyController ...")

        rospy.wait_for_service('/head_pan_joint/relax')
        rospy.wait_for_service('/head_tilt_joint/relax')
        rospy.wait_for_service('/left_elbow/relax')
        rospy.wait_for_service('/left_sho_pitch/relax')
        rospy.wait_for_service('/left_sho_roll/relax')
        rospy.wait_for_service('/right_elbow/relax')
        rospy.wait_for_service('/right_sho_pitch/relax')
        rospy.wait_for_service('/right_sho_roll/relax')
        rospy.wait_for_service('/head_pan_joint/enable')
        rospy.wait_for_service('/head_tilt_joint/enable')
        rospy.wait_for_service('/left_elbow/enable')
        rospy.wait_for_service('/left_sho_pitch/enable')
        rospy.wait_for_service('/left_sho_roll/enable')
        rospy.wait_for_service('/right_elbow/enable')
        rospy.wait_for_service('/right_sho_pitch/enable')
        rospy.wait_for_service('/right_sho_roll/enable')        

        self.relax_servos()
        self.enable_servos()

        if self.pages:
            l = len(self.pages)
        else:
            l = 0

        flag = False
        n = 0
 

        while not rospy.is_shutdown():

            # Wait until initial pose is captured
            if not self.ready:
                rospy.loginfo("Not ready ...")
                continue

            if l > 1:
                n = r.randint(0, l-1)
            else:
                n = 0
            # xposes = Poses(self.pages[n]) 
            # rospy.loginfo("**-------\nPlaying page: %s" % xposes.getTitle()) # print title
            # rospy.loginfo("**-------\nMotion: %s" % xposes.getPoses())
            xposes = Poses()

            # if not self.pages and not self.poses:
            #     continue
            if self.pages:
                xposes.loadPage(self.pages[n])
            elif self.poses:
                xposes.setTitle(self.poseTitle)
                xposes.loadPoses(self.poses)
            else:
                continue            
            
            # xposes.loadPage(self.pages[n])    # for random
            # xposes.setPoses(self.poses)
            
            # Only add initial pose at the very beginning - when the node first starts
            if not self.add_initial:
                rospy.loginfo("INITIAL JOINT POSE: %s" % self.joint_pos)
                xposes.addInitialPose(self.joint_pos)
                self.add_initial = True
            poses = xposes.getPoses()   # Load poses
            timing = xposes.getTiming()
            
            new_poses = {}
            posex_length = 0

            if self.ready:
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
                    # else:[1.9739583333333333, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0, 15.5, 16.0, 16.5, 17.0, 17.5, 18.0, 18.5, 19.0, 19.5, 20.0, 20.5, 21.0, 21.5, 22.0, 22.5, 23.0, 23.5, 24.0, 24.5, 25.0, 25.5, 26.0, 26.5, 27.0, 27.5, 28.0, 28.5, 29.0, 29.5, 30.0, 30.5, 31.0, 31.5, 32.0, 32.5, 33.0, 33.5, 34.0, 34.5, 35.0, 35.5, 36.0, 36.5, 37.0, 37.5, 38.0, 38.5, 39.0, 39.5, 40.0, 40.5, 41.0, 41.5, 42.0, 42.5, 43.0, 43.5, 44.0, 44.5, 45.0, 45.5, 46.0, 46.5, 47.0, 47.5, 48.0, 48.5, 49.0, 49.5, 50.0, 50.5, 51.0, 51.5, 52.0, 52.5, 53.0, 53.5, 54.0, 54.5, 55.0, 55.5, 56.0, 56.5, 57.0, 57.5, 58.0, 58.5, 59.0, 59.5, 60.0, 60.5, 61.0, 61.5, 62.0, 62.5, 63.0, 63.5, 64.0, 64.5, 65.0, 65.5, 66.0, 66.5, 67.0, 67.5, 68.0, 68.5, 69.0, 69.5, 70.0, 70.5, 71.0, 71.5, 72.0, 72.5, 73.0, 73.5, 74.0, 74.5, 75.0, 75.5, 76.0, 76.5, 77.0, 77.5, 78.0, 78.5, 79.0, 79.5, 80.0, 80.5, 81.0, 81.5, 82.0, 82.5, 83.0, 83.5, 84.0, 84.5, 85.0, 85.5, 86.0, 86.5, 87.0, 87.5, 88.0, 88.5, 89.0, 89.5, 90.0, 90.5, 91.0, 91.5, 92.0, 92.5, 93.0, 93.5, 94.0, 94.5, 95.0, 95.5, 96.0, 96.5, 97.0, 97.5, 98.0, 98.5, 99.0, 99.5, 100.0, 100.5, 101.0, 101.5, 102.0, 102.5, 103.0, 103.5, 104.0, 104.5, 105.0, 105.5, 106.0, 106.5, 107.0, 107.5, 108.0, 108.5, 109.0, 109.5, 110.0, 110.5, 111.0, 111.5, 112.0, 112.5, 113.0, 113.5, 114.0, 114.5, 115.0, 115.5, 116.0, 116.5, 117.0, 117.5, 118.0, 118.5, 119.0, 119.5, 120.0, 120.5, 121.0, 121.5, 122.0, 122.5, 123.0, 123.5, 124.0, 124.5, 125.0, 125.5, 126.0, 126.5, 127.0, 127.5, 128.0, 128.5, 129.0, 129.5, 130.0, 130.5, 131.0, 131.5, 132.0, 132.5, 133.0, 133.5, 134.0, 134.5, 135.0, 135.5, 136.0, 136.5, 137.0, 137.5, 138.0, 138.5, 139.0, 139.5, 140.0, 140.5, 141.0, 141.5, 142.0, 142.5, 143.0, 143.5, 144.0, 144.5, 145.0, 145.5, 146.0, 146.5, 147.0, 147.5, 148.0, 148.5, 149.0, 149.5, 150.0, 150.5, 151.0, 151.5, 152.0, 152.5, 153.0, 153.5, 154.0, 154.5, 155.0, 155.5, 156.0, 156.5, 157.0, 157.5, 158.0, 158.5, 159.0, 159.5, 160.0, 160.5, 161.0, 161.5, 162.0, 162.5, 163.0, 163.5, 164.0, 164.5, 165.0, 165.5, 166.0, 166.5, 167.0, 167.5, 168.0, 168.5, 169.0, 169.5, 170.0, 170.5, 171.0, 171.5, 172.0, 172.5, 173.0, 173.5, 174.0, 174.5, 175.0, 175.5, 176.0, 176.5, 177.0, 177.5, 178.0, 178.5, 179.0, 179.5, 180.0, 180.5, 181.0, 181.5, 182.0, 182.5, 183.0, 183.5, 184.0, 184.5, 185.0, 185.5, 186.0, 186.5, 187.0, 187.5, 188.0, 188.5, 189.0, 189.5, 190.0, 190.5, 191.0, 191.5, 192.0, 192.5, 193.0, 193.5, 194.0, 194.5, 195.0, 195.5, 196.0, 196.5, 197.0, 197.5, 198.0, 198.5, 199.0, 199.5, 200.0, 200.5, 201.0, 201.5, 202.0, 202.5, 203.0, 203.5, 204.0, 204.5, 205.0, 205.5, 206.0, 206.5, 207.0, 207.5, 208.0, 208.5, 209.0, 209.5, 210.0, 210.5, 211.0, 211.5, 212.0, 212.5, 213.0, 213.5, 214.0, 214.5, 215.0, 215.5, 216.0, 216.5, 217.0, 217.5, 218.0, 218.5, 219.0, 219.5, 220.0, 220.5, 221.0, 221.5, 222.0, 222.5, 223.0, 223.5, 224.0, 224.5, 225.0, 225.5, 226.0, 226.5, 227.0, 227.5, 228.0, 228.5, 229.0, 229.5, 230.0, 230.5, 231.0, 231.5, 232.0, 232.5, 233.0, 233.5, 234.0, 234.5, 235.0, 235.5, 236.0, 236.5, 237.0, 237.5, 238.0, 238.5, 239.0, 239.5, 240.0, 240.5, 241.0, 241.5, 242.0, 242.5, 243.0, 243.5, 244.0, 244.5, 245.0, 245.5, 246.0, 246.5, 247.0, 247.5, 248.0, 248.5, 249.0, 249.5, 250.0, 250.5, 251.0, 251.5, 252.0, 252.5, 253.0, 253.5, 254.0, 254.5, 255.0, 255.5, 256.0, 256.5, 257.3333333333333, 257.984375, 258.0, 258.5, 259.0, 259.5, 260.0, 260.5, 261.0, 261.5, 262.0, 262.5, 263.0, 263.5, 264.0, 264.5, 265.0, 265.5, 266.0, 266.5, 267.0, 267.5, 268.0, 268.5, 269.0, 269.5, 270.0, 270.5, 271.0, 271.5, 272.0, 272.5, 273.0, 273.5, 274.0104166666667, 274.5, 275.0104166666667, 275.5, 276.0, 276.5, 277.0, 277.5, 278.0052083333333, 278.5, 279.28125, 279.5, 280.3020833333333, 280.5]
                    #     posex = list(_posex) 
                    #     timing['Timer'] = list(timing['Time'])

                    # # Version 1.5
                    if self.pausemode:
                        posex = []
                        timer = []
                        pauses = timing['PauseTime']
                        for m in range(len(timing['PauseTime'])):
                            # The number of frames to embed = PauseTime/100
                            steps = _timer[m] * 8.0/50
                            # mult = int((round(pauses[m]/100.0)-1))
                            mult = int(round(pauses[m] / (3 * steps)))
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
                # rospy.loginfo("p/pause: %s/%s" % (p, pause))
                

                # if self.write_to_file and not self.pausemode:
                #     try:
                #         pausetime_fh = open(file_pausetime, 'w')
                #         pausetime_fh.write("PauseTime\n\n")
                #     except IOError as e:
                #         print "Problem opening file %s\nCause: %s" % (file_pausetime, e)
                if self.loglog and not self.pausemode:
                    rospy.loginfo("LOGLOG: PauseTime data: %s" % pause)

                # Execute all motion until finished
                for i in range(posex_length):
                    mx = posex_length/p + 1

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
                        
                    while rospy.get_time() - self.then < d:                                         
                        pass
                    self.then = rospy.get_time()

                    # Quit/immediately stop if interrupted
                    if not self.ready:
                        break

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


            self.ready = False
            self.add_initial = False
            self.pages = None
            self.poses = None
            self.sleeper.sleep()
            # rospy.signal_shutdown("Only running once.")
            # return 0

    def chooseInterpolate(self, posex, timing, interp='cubic'):
        if interp == 'cubic' or interp == 'lagrange' or interp=='linear' or interp == 'catmull-rom':
            return self.interpolate2(posex, timing, interp)
        # elif interp == 'linear':
        #     return self.interpolate(posex, timing)
        elif interp == None:
            return None
        else:
            raise ValueError("Invalid interpolation type")


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

    def interpolate2(self, fubar, timing, interp):
        l = len(fubar)
        x = np.linspace(0, l-1, num=l, endpoint=True)
        y = fubar
        # f = interp1d(x,y)

        steps = 0
        for t in timing['Timer']:
            steps += int(t * 8/50.0)

        xnew = np.linspace(0, l-1, num=steps, endpoint=True)
        rospy.loginfo("Interpolate2 steps: %s" % steps)
        if interp == 'cubic' or interp == 'linear':
            f2 = interp1d(x,y, kind=interp)
        elif interp == 'lagrange':
            f2 = lagrange(x, y)
        elif interp == 'catmull-rom':
            x_intpol, y_intpol = self.catmull_rom(x, y, steps)
            plt.figure()
            plt.scatter(x, y)
            plt.plot(x_intpol, y_intpol)
            plt.show()
            return [self.mapp(f) for f in y_intpol]
        else:
            raise ValueError('Invalid interpolation type!')
        

        fx = [self.mapp(f) for f in f2(xnew)]
        plt.figure()
        plt.scatter(x, y)
        plt.plot(xnew, f2(xnew))
        plt.show()

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
    def catmull_rom_one_point(self, x, v0, v1, v2, v3):
        """Computes interpolated y-coord for given x-coord using Catmull-Rom.

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
        c1 = 1. * v1
        c2 = -.5 * v0 + .5 * v2
        c3 = 1. * v0 + -2.5 * v1 + 2. * v2 -.5 * v3
        c4 = -.5 * v0 + 1.5 * v1 + -1.5 * v2 + .5 * v3
        return (((c4 * x + c3) * x + c2) * x + c1)

    def catmull_rom(self, p_x, p_y, res):
        """Computes Catmull-Rom Spline for given support points and resolution.

        Args:
            p_x: array of x-coords
            p_y: array of y-coords
            res: resolution of a segment (including the start point, but not the
                endpoint of the segment)
        """
        res = int(ceil(self.catmull_rom_res * res))
        # res = self.catmull_rom_res
        # create arrays for spline points
        x_intpol = np.empty(res*(len(p_x)-1) + 1)
        y_intpol = np.empty(res*(len(p_x)-1) + 1)

        # set the last x- and y-coord, the others will be set in the loop
        x_intpol[-1] = p_x[-1]
        y_intpol[-1] = p_y[-1]

        # loop over segments (we have n-1 segments for n points)
        for i in range(len(p_x)-1):
            # set x-coords
            x_intpol[i*res:(i+1)*res] = np.linspace(
                p_x[i], p_x[i+1], res, endpoint=False)
            if i == 0:
                # need to estimate an additional support point before the first
                y_intpol[:res] = np.array([
                    self.catmull_rom_one_point(
                        x,
                        p_y[0] - (p_y[1] - p_y[0]), # estimated start point,
                        p_y[0],
                        p_y[1],
                        p_y[2])
                    for x in np.linspace(0.,1.,res, endpoint=False)])
            elif i == len(p_x) - 2:
                # need to estimate an additional support point after the last
                y_intpol[i*res:-1] = np.array([
                    self.catmull_rom_one_point(
                        x,
                        p_y[i-1],
                        p_y[i],
                        p_y[i+1],
                        p_y[i+1] + (p_y[i+1] - p_y[i]) # estimated end point
                    ) for x in np.linspace(0.,1.,res, endpoint=False)])
            else:
                y_intpol[i*res:(i+1)*res] = np.array([
                    self.catmull_rom_one_point(
                        x,
                        p_y[i-1],
                        p_y[i],
                        p_y[i+1],
                        p_y[i+2]) for x in np.linspace(0.,1.,res, endpoint=False)])


        return (x_intpol, y_intpol)

def main(args):
    rospy.init_node('jimmy_controller_node_v2', anonymous=True)
    jimmyc = JimmyController()
    jimmyc.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)

