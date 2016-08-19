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
from std_msgs.msg import Float32, Float64, String, Bool

from arbotix_msgs.srv import *
from rebel_ros.srv import *

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

        rospy.wait_for_service('rebel_parser_server')
        self.rebel_parser = rospy.ServiceProxy('rebel_parser_server', Rebel)

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
        self.init_ready = [False]*8
        self.ready = False
        self.enable_ready = False
        self.add_initial = False # False: add initial pose, True: don't add initial pose        
        self.pausemode = True  # False: hard stops, True: blended in interpolated data
        self.interpolation_mode = 'cubic'

        # self.write_to_file = True
        self.loglog = True

        # self.interpolate_fh = None
        # self.timer_fh = None
        # self.pausetime_fh = None

        rospack = rospkg.RosPack()

        # Load from .pagelist file
        # tree = etree.parse('/home/mathias/Downloads/WinRME/Chair-Poses.pagelist')
        # tree = etree.parse('%s/src/pagelists/test-poses.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/src/pagelists/Chair-Poses.pagelist' % rospack.get_path('arbotix_controller'))
        tree = etree.parse('%s/src/pagelists/marie_curie2_edited4.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/src/pagelists/PositionSequence4.pagelist' % rospack.get_path('arbotix_controller'))
        # tree = etree.parse('%s/PositionSequence.pagelist' % rospack.get_path('rebel_ros'))
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

    def gesture_callback(self, gesture):
        # Reload the motion file if new gesture is given
        rospy.loginfo("[gesture_callback] Got new gesture: %s" % gesture.data)
        print "Greeting.data: %s" % gesture.data
        get = self.rebel_parser("greeting")
        word = get.word
        sequence = get.sequence
        rospy.loginfo("[gesture_callback] response:\n%s\n%s" % (word, sequence))
        self.poses = sequence
        self.ready = True
        # tree = etree.parse('%s/PositionSequence.pagelist' % rospack.get_path('rebel_ros'))
        # self.pages = tree.findall('.//PageClass')
        # self.page_length = len(self.pages)
        # self.ready = True

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

            if not self.pages and not self.poses:
                continue
            
            xposes = Poses()
            xposes.loadPage(self.pages[n])    # for random
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
            self.pages = None
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

def main(args):
    rospy.init_node('jimmy_controller_node_v2', anonymous=True)
    jimmyc = JimmyController()
    jimmyc.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)

