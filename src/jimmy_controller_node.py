#!/usr/bin/env python
import sys, threading
import random as r
import xml.etree.ElementTree as etree
import decimal
from math import *
from collections import deque

from scipy.interpolate import interp1d, lagrange
import numpy as np
import matplotlib.pyplot as plt

import rospy
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64

from arbotix_msgs.srv import *

from poses import Poses

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

        self.init_ready = [False]*8
        self.ready = False
        self.enable_ready = False
        self.add_initial = False    # False: add initial pose, True: don't add initial pose

        rospack = rospkg.RosPack()

        # Load from .pagelist file
        # tree = etree.parse('/home/mathias/Downloads/WinRME/Chair-Poses.pagelist')
        # tree = etree.parse('%s/src/pagelists/test-poses.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/src/pagelists/Chair-Poses.pagelist' % rospack.get_path('arbotix_controller'))
        tree = etree.parse('%s/src/pagelists/marie_curie2_edited4.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('/home/mathias/Projects/jimmy_ros/src/arbotix_controller/src/PositionSequence.pagelist')
        self.pages = tree.findall('.//PageClass')
        self.page_length = len(self.pages)

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


        pausemode = True  # blended in interpolated data
        # pausemode = False # hard stops

        l = len(self.pages)

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
            
            xposes = Poses(self.pages[n]) # for random
            
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
                for joint, pub in self.joints.iteritems():
                    rospy.loginfo("Joint: %s" % joint)
                    _posex = poses[joint]
                    _timer = timing['Time']
                    
                    # If pausemode true: embed frames into motion data
                    if pausemode:
                        posex = []
                        timer = []
                        pauses = timing['PauseTime']
                        for m in range(len(timing['PauseTime'])):
                            # The number of frames to embed = PauseTime/100
                            mult = int((round(pauses[m]/100.0)-1))
                            posex += [[_posex[m]] + [_posex[m]]*mult]
                            timer += [[_timer[m]] + [_timer[m]]*mult]
                        posex = [f for s in posex for f in s]
                        timer = [t for j in timer for t in j]
                        timing['Timer'] = timer
                    else:
                        posex = list(_posex) 
                        timing['Timer'] = list(timing['Time'])


                    # Choose interpolation method: linear, cubic, or lagrange
                    # So far lagrange has a lot of issues (movements too big)
                    fposex = self.chooseInterpolate(posex, timing, 'cubic')
                    # rospy.loginfo("lagrange fPosex[%s]: %s" % (joint, fposex))
                    new_poses[joint] = fposex
                    posex_length = len(fposex)

                p = len(timing['Timer'])
                # p = len(timing['PauseTime'])
                if not pausemode:
                    pause = [t * 0.01 for t in timing['PauseTime']]
                # rospy.loginfo("p/pause: %s/%s" % (p, pause))
                

                for i in range(posex_length):
                    mx = posex_length/p + 1

                    # Uncomment to move step by step
                    # if (i % mx == 0):
                    #     raw_input("Press Enter to step through...")       
                   
                    for joint, pub in self.joints.iteritems():
                        pos = new_poses[joint][i]
                        pub.publish(Float64(pos))

                    d = 0.02
                    
                    if (i % mx == 0) and not pausemode:
                        ind = i/mx
                        d += pause[ind] 
                        rospy.loginfo("Wait for: %s (%s) " % (d, pause[ind]*100))
                    while rospy.get_time() - self.then < d:                                         
                        pass
                    self.then = rospy.get_time()

            self.sleeper.sleep() 

    def chooseInterpolate(self, posex, timing, interp='cubic'):
        if interp == 'cubic' or interp == 'lagrange':
            return self.interpolate2(posex, timing, interp)
        elif interp == 'linear':
            return self.interpolate(posex, timing)
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
        if interp == 'cubic':
            f2 = interp1d(x,y, kind='cubic')
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

def main(args):
    rospy.init_node('jimmy_controller_node', anonymous=True)
    jimmyc = JimmyController()
    jimmyc.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)

