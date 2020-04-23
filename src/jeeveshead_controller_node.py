#!/usr/bin/env python
import sys, threading
import random as r
import xml.etree.ElementTree as etree
import decimal
from math import *
import ast
from collections import deque

from scipy.interpolate import interp1d, lagrange
import numpy as np
import matplotlib.pyplot as plt

import rospy
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64, String

from arbotix_msgs.srv import *
from rebel_ros.srv import *

from poses import Poses, JeevesHeadPoses


class JeevesHeadController(threading.Thread):
    roll = 0.0
    pan = 0.0
    then = 0

    def __init__(self):

        # Publishers
        self.tilt_right_pub = rospy.Publisher('/tilt_right/command', Float64, queue_size=10)
        self.tilt_left_pub = rospy.Publisher('/tilt_left/command', Float64, queue_size=10)
        self.pan_pub = rospy.Publisher('/pan/command', Float64, queue_size=10)
        self.face_pub = rospy.Publisher('/face', String, queue_size=10, latch=True)

        # Subscribers
        # Arbotix joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        # REBeL entry point - send a REBeL expression to this topic
        rospy.Subscriber('/gesture_command', String, self.gesture_callback)

        # Services
        self.tilt_right_relax = rospy.ServiceProxy(
            '/tilt_right/relax', Relax)
        self.tilt_left_relax = rospy.ServiceProxy(
            '/tilt_left/relax', Relax)
        self.pan_relax = rospy.ServiceProxy(
            '/pan/relax', Relax)

        self.tilt_right_enable = rospy.ServiceProxy(
            '/tilt_right/enable', Enable)
        self.tilt_left_enable = rospy.ServiceProxy(
            '/tilt_left/enable', Enable)
        self.pan_enable = rospy.ServiceProxy(
            '/pan/enable', Enable)

        self.tilt_right_set_slope_cw = rospy.ServiceProxy('/tilt_right/set_compliance_slope_cw', ComplianceSlopeCW)
        self.tilt_right_set_slope_ccw = rospy.ServiceProxy('/tilt_right/set_compliance_slope_ccw', ComplianceSlopeCCW)

        self.tilt_left_set_slope_cw = rospy.ServiceProxy('/tilt_left/set_compliance_slope_cw', ComplianceSlopeCW)
        self.tilt_left_set_slope_ccw = rospy.ServiceProxy('/tilt_left/set_compliance_slope_ccw', ComplianceSlopeCCW)

        self.pan_set_slope_cw = rospy.ServiceProxy('/pan/set_compliance_slope_cw', ComplianceSlopeCW)
        self.pan_set_slope_ccw = rospy.ServiceProxy('/pan/set_compliance_slope_ccw', ComplianceSlopeCCW)

        # REBeL service
        rospy.wait_for_service('jeeves_rebel_server')
        self.rebel_parser = rospy.ServiceProxy('jeeves_rebel_server', Rebel)

        self.joints = {'TILT_LEFT': self.tilt_left_pub,
            'TILT_RIGHT': self.tilt_right_pub,
            'PAN': self.pan_pub,
            'FACE_CMD': self.face_pub}

        self.joint_pos = {'TILT_LEFT': 512,
            'TILT_LEFT': 512,
            'PAN': 512,
            'FACE_CMD': ''}

        self.init_ready = [False, False, False]
        self.ready = False
        self.enable_ready = False
        self.poses = None
        self.poseTitle = "Default"
        # self.add_initial = False # False: add initial pose, True: don't add initial pose
        self.pausemode = True  # False: hard stops, True: blended in interpolated data
        self.interpolation_mode = 'linear'
        self.idle_time = rospy.get_time()
        self.then = rospy.get_time()

        # # self.write_to_file = True
        self.loglog = True
        self.action = []
        self.slope_val = 1.0

        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(20)

    # Service methods
    def relax_servos(self):
        self.tilt_right_relax()
        self.tilt_left_relax()
        self.pan_relax()

    def enable_servos(self):
        self.tilt_right_enable(True)
        self.tilt_left_enable(True)
        self.pan_enable(True)
        self.enable_ready = True

    def set_slopes(self):
        slope = self.slope_val
        try:
            rospy.loginfo("Settings slope to: %s" % slope)
            self.tilt_right_set_slope_ccw(slope)
            self.tilt_right_set_slope_cw(slope)
            self.tilt_left_set_slope_ccw(slope)
            self.tilt_left_set_slope_cw(slope)
            self.pan_set_slope_ccw(slope)
            self.pan_set_slope_cw(slope)

        except Exception as se:
            rospy.logerr("Something went wrong calling slope service: {}".format(se))
            pass

    # Callback methods
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
                if joint_name == 'tilt_right':
                    self.joint_pos['TILT_RIGHT'] = joint_pos
                    self.init_ready[0] = True
                    rospy.loginfo(">>>>> TILT_RIGHT (%s)" % joint_pos)
                if joint_name == 'tilt_left':
                    self.joint_pos['TILT_LEFT'] = joint_pos
                    self.init_ready[1] = True
                    rospy.loginfo(">>>>> TILT_LEFT (%s)" % joint_pos)
                if joint_name == 'pan':
                    self.joint_pos['PAN'] = joint_pos
                    self.init_ready[2] = True
                    rospy.loginfo(">>>>> PAN (%s)" % joint_pos)

        if all(self.init_ready):
            if not self.ready:
                rospy.loginfo("OK! I'm ready")
            self.ready = True

    def gesture_callback(self, gesture):
        '''
        Callback for /gesture_command topic
        gesture (str): REBeL expression to parse and execute
        '''
        caller = "[gesture_callback]"
        # Reload the motion file if new gesture is given
        rospy.loginfo("[gesture_callback] Got new gesture: %s" % gesture.data)
        #print "Greeting.data: %s" % gesture.data
        get = self.rebel_parser(gesture.data)
        word = get.word
        sequence = get.sequence

        # if self.use_midi:
        #     rospy.loginfo("%s med_file_path: %s" % (caller, self.med_file_path))
        #     mididata = self.midi_motion(self.med_file_path)


        if word:
            self.poseTitle = gesture.data
            seq = ast.literal_eval(sequence)

        #     if self.use_midi:
        #         rospy.loginfo("%s Applying MIDI data from: ... %s" % (caller, self.midi_title))
        #         rospy.loginfo("%s original timing 'Time': %s" % (caller, seq['Time']))
        #         seq['PauseTime'] = self.update_timing_data(seq['PauseTime'], ast.literal_eval(mididata.timing), mul=200)
        #         seq['Time'] = self.update_timing_data(seq['Time'], ast.literal_eval(mididata.timing), mul=200)
        #         rospy.loginfo("%s updated timing 'Time': %s" % (caller, seq['Time']))
        #     rospy.loginfo("%s response:\n%s\n%s" % (caller, word, seq))

            rospy.loginfo("[gesture_callback] response:\n%s\n%s" % (word, seq))
            self.poses = seq
            self.ready = True
            self.after_motion = False
            self.gesture_command = True
            rospy.loginfo("OK! I'm ready to execute: {}".format(self.poseTitle))

    def resetPose(self):
        rospy.loginfo("***ResetPoses***")
        # self.set_speed('slow')
        wait_time = abs(np.random.normal(loc=2.0, scale=1))
        while rospy.get_time() - self.idle_time < wait_time:
            continue
        for joint, pub in self.joints.items():
            # pub.publish(Float64(self.joint_pos[joint]))
            if joint != 'FACE_CMD':
                pub.publish(Float64(0.0))
            else:
                pub.publish(String(np.random.choice(['w','q','e'])))


    def pospos(self, x):
        # Convert joint_pos from -2.6 - 2.6 to 0 - 1020
        return min(1020, max(0, int((x + 2.6)/2.6 * 512)))

    def run(self):
        rospy.loginfo("Setting up JeevesHeadController ...")

        rospy.wait_for_service('/tilt_right/relax')
        rospy.wait_for_service('/tilt_left/relax')
        rospy.wait_for_service('/pan/relax')
        rospy.wait_for_service('/tilt_right/enable')
        rospy.wait_for_service('/tilt_left/enable')
        rospy.wait_for_service('/pan/enable')


        self.relax_servos()
        self.enable_servos()
        self.set_slopes()

        flag = False
        n = 0

        new_poses = {}
        posex_length = 0

        while not rospy.is_shutdown():


            # Wait until initial pose is captured
            if not self.ready:
                rospy.loginfo("Not ready ...")
                continue

            # if l > 1:
            #     n = r.randint(0, l-1)
            # else:
            #     n = 0
            # xposes = Poses(self.pages[n])
            # rospy.loginfo("**-------\nPlaying page: %s" % xposes.getTitle()) # print title
            # rospy.loginfo("**-------\nMotion: %s" % xposes.getPoses())

            # xposes = Poses(self.pages[n]) # for random

            # Only add initial pose at the very beginning - when the node first starts
            # if not self.add_initial:
            #     rospy.loginfo("INITIAL JOINT POSE: %s" % self.joint_pos)
            #     xposes.addInitialPose(self.joint_pos)
            #     self.add_initial = True

            # new_poses = {}
            # posex_length = 0

            if self.ready and self.poses:
                xposes = JeevesHeadPoses()
                xposes.setTitle(self.poseTitle)
                xposes.loadPoses(self.poses)
                poses = xposes.getPoses()   # Load poses
                timing = xposes.getTiming()

                # rospy.loginfo("Going to do %s!!!" % xposes.getTitle())

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


                '''
                For each joint:
                - pausemode: calculate how much additional frames must be added per pause/time information
                - interpolate: interpolate using the chosen interpolation parameters
                '''
                for joint, pub in self.joints.iteritems():
                    rospy.loginfo("Joint: %s" % joint)
                    _posex = poses[joint]
                    _timer = timing['Time']

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
                    if joint == 'FACE_CMD':
                        fposex = self.face_cmd_interpolate(posex, timing, self.interpolation_mode)
                        rospy.loginfo("POSEX: {}".format(fposex))
                    else:
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


                if self.loglog and not self.pausemode:
                    rospy.loginfo("LOGLOG: PauseTime data: %s" % pause)

                # Execute motion for each timestep
                for i in range(posex_length):
                    mx = posex_length/p + 1

                    # Uncomment to move step by step
                    # if (i % mx == 0):
                    #     ind = i/mx
                    #     rospy.loginfo("LOGLOLG: PauseTime: %s" % pauses[ind])
                    #     raw_input("Press Enter to step through...")

                    ###  Publish joint data for each joint <MAIN EVENT> ###
                    for joint, pub in self.joints.iteritems():
                        pos = new_poses[joint][i]
                        print("Publishing: ", pos)
                        # pub.publish(Float64(pos))

                        if joint != 'FACE_CMD':
                            pos = Float64(pos)
                        else:
                            pos = String(pos)

                        pub.publish(pos)

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

                self.resetPose()
                # self.relax_servos()
                then = rospy.get_time()
                while rospy.get_time() - then < 1.5:
                    pass
                # self.enable_servos()
                self.set_slopes()


            # Reset things
            posex_length = 0
            # self.ready = False
            self.idle_time = rospy.get_time()  # Reset idle timer
            self.pages = None
            self.poses = None
            new_poses = {}
            self.sleeper.sleep()
            # rospy.signal_shutdown("Only running once.")
            # return 0

    def chooseInterpolate(self, posex, timing, interp='cubic'):
        if interp == 'cubic' or interp == 'lagrange' or interp=='linear':
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
        if interp == 'cubic' or interp == 'linear':
            f2 = interp1d(x,y, kind=interp)
        elif interp == 'lagrange':
            f2 = lagrange(x, y)
        else:
            raise ValueError('Invalid interpolation type!')
        steps = 0
        for t in timing['Timer']:
            steps += int(t * 8/50.0)

        xnew = np.linspace(0, l-1, num=steps, endpoint=True)

        fx = [self.mapp(f) for f in f2(xnew)]

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

    def face_cmd_interpolate(self, posex, timing, interp):
        l = len(posex)
        x = np.linspace(0, l-1, num=l, endpoint=True)
        y = np.ones((l,))
        # f = interp1d(x,y)
        if interp == 'cubic' or interp == 'linear':
            f2 = interp1d(x,y, kind=interp)
        elif interp == 'lagrange':
            f2 = lagrange(x, y)
        else:
            raise ValueError('Invalid interpolation type!')
        steps = 0
        for t in timing['Timer']:
            steps += int(t * 8/50.0)

        xnew = np.linspace(0, l-1, num=steps, endpoint=True)

        fx = [posex[-1] for f in f2(xnew)]

        return fx

def main(args):
    rospy.init_node('jeeveshead_controller_node', anonymous=True)
    jeevesh = JeevesHeadController()
    jeevesh.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)

