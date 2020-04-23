from collections import defaultdict
import rospy

class BasePoses(object):

    body = None
    init_body = None
    timing = {'Time': None,
              'PauseTime': None
             }
    title = "Default"

    def __init__(self, page=None, body=None, init_body=None):
        self.body = body
        self.init_body = {joint: 512 for joint in self.body.keys()}
        if page:
            self.loadPage(page)

    def loadPage(self, page):

        self.poses = page.find('Poses')
        self.poseclasses = [pc.findall('.//PoseClass') for pc in self.poses]
        # self.poses = page.find('Poses')
        self.title = page.find('Title').text

        # self.allpages = []

        self.extractPoses()

    def extractPoses(self):
#         self.pose_classes = [pc for pc in self.poses]
#         print self.poses[0]
#         print self.pose_classes
#         for pose_class in self.poses:
#             print pose_class

        # fbody = defaultdict(list)
        for key in self.body.keys():
            self.body[key] = self._getJointPoses(key, self.poses)

        for key in self.timing.keys():
            self.timing[key] = self._getJointPoses(key, self.poses)

        # return allpages

    def loadPoses(self, sequence):
        rospy.loginfo("[loadPoses] Sequence: %s" % type(sequence))
        for key, value in sequence.iteritems():
            if key in self.body.keys():
                self.body[key] = value
            elif key in self.timing.keys():
                self.timing[key] = value

    def _getJointPoses(self, joint_name, poses):
        return [int(pcx.find(joint_name).text) for pcx in poses]

    def setTitle(self, title):
        self.title = title

    def getTitle(self):
        return self.title

    def getPoses(self):
        return self.body

    def getTiming(self):
        return self.timing

    def addInitialPose(self, init_pose, init_timing=None):

        i_timing = {'Time': 550,
                    'PauseTime': 100
                    }
        for key, value in self.init_body.iteritems():
            if self.body[key] == None:
                self.body[key] = [value]
            else:
                self.body[key].insert(0, value)

        for key, value in i_timing.iteritems():
            self.timing[key].insert(0, value)
            self.timing[key].insert(0, value)

        for key, value in init_pose.iteritems():
            self.body[key] = [value]*2 + self.body[key][1:]

        if init_timing != None:
            for key, value in init_timing.iteritems():
                self.timing[key][0] = [value]*2 + self.timing[key][2:]


class Poses:
    def __init__(self, page=None):
        self.body = {'R_SHO_PITCH': None,
                    'L_SHO_PITCH': None,
                    'R_SHO_ROLL': None,
                    'L_SHO_ROLL': None,
                    'R_ELBOW': None,
                    'L_ELBOW': None,
                    'R_HIP_YAW': None,
                    'L_HIP_YAW': None,
                    'R_HIP_ROLL': None,
                    'L_HIP_ROLL': None,
                    'R_HIP_PITCH': None,
                    'L_HIP_PITCH': None,
                    'R_KNEE': None,
                    'L_KNEE': None,
                    'R_ANK_PITCH': None,
                    'L_ANK_PITCH': None,
                    'R_ANK_ROLL': None,
                    'L_ANK_ROLL': None,
                    'HEAD_PAN': None,
                    'HEAD_TILT': None
                     }

        self.timing = {'Time': None,
                    'PauseTime': None
                    }
        self.title = "Default"
        if page:
            self.loadPage(page)

    def loadPage(self, page):

        self.poses = page.find('Poses')
        self.poseclasses = [pc.findall('.//PoseClass') for pc in self.poses]
        # self.poses = page.find('Poses')
        self.title = page.find('Title').text

        # self.allpages = []

        self.extractPoses()

    def extractPoses(self):
#         self.pose_classes = [pc for pc in self.poses]
#         print self.poses[0]
#         print self.pose_classes
#         for pose_class in self.poses:
#             print pose_class

        # fbody = defaultdict(list)
        for key in self.body.keys():
            self.body[key] = self._getJointPoses(key, self.poses)

        for key in self.timing.keys():
            self.timing[key] = self._getJointPoses(key, self.poses)

        # return allpages

    def loadPoses(self, sequence):
        rospy.loginfo("[loadPoses] Sequence: %s" % type(sequence))
        for key, value in sequence.iteritems():
            if key in self.body.keys():
                self.body[key] = value
            elif key in self.timing.keys():
                self.timing[key] = value

    def _getJointPoses(self, joint_name, poses):
        return [int(pcx.find(joint_name).text) for pcx in poses]

    def setTitle(self, title):
        self.title = title

    def getTitle(self):
        return self.title

    def getPoses(self):
        return self.body

    def getTiming(self):
        return self.timing

    def addInitialPose(self, init_pose, init_timing=None):
        init_body = {'R_SHO_PITCH': 512,
                    'L_SHO_PITCH': 512,
                    'R_SHO_ROLL': 512,
                    'L_SHO_ROLL': 512,
                    'R_ELBOW': 512,
                    'L_ELBOW': 512,
                    'R_HIP_YAW': 512,
                    'L_HIP_YAW': 512,
                    'R_HIP_ROLL': 512,
                    'L_HIP_ROLL': 512,
                    'R_HIP_PITCH': 512,
                    'L_HIP_PITCH': 512,
                    'R_KNEE': 512,
                    'L_KNEE': 512,
                    'R_ANK_PITCH': 512,
                    'L_ANK_PITCH': 512,
                    'R_ANK_ROLL': 512,
                    'L_ANK_ROLL': 512,
                    'HEAD_PAN': 512,
                    'HEAD_TILT': 512
                     }
        i_timing = {'Time': 550,
                    'PauseTime': 100
                    }
        for key, value in init_body.iteritems():
            if self.body[key] == None:
                self.body[key] = [value]
            else:
                self.body[key].insert(0, value)

        for key, value in i_timing.iteritems():
            self.timing[key].insert(0, value)
            self.timing[key].insert(0, value)

        for key, value in init_pose.iteritems():
            self.body[key] = [value]*2 + self.body[key][1:]

        if init_timing != None:
            for key, value in init_timing.iteritems():
                self.timing[key][0] = [value]*2 + self.timing[key][2:]

# For use with jimmy torso v2 (with elbow roll joints)
class Poses2:
    def __init__(self):
        self.body = {'R_SHO_PITCH': None,
                    'L_SHO_PITCH': None,
                    'R_SHO_ROLL': None,
                    'L_SHO_ROLL': None,
                    'R_ELBOW_ROLL': None,
                    'R_ELBOW_PITCH': None,
                    'L_ELBOW_ROLL': None,
                    'L_ELBOW_PITCH': None,
                    'R_HIP_YAW': None,
                    'L_HIP_YAW': None,
                    'R_HIP_ROLL': None,
                    'L_HIP_ROLL': None,
                    'R_HIP_PITCH': None,
                    'L_HIP_PITCH': None,
                    'R_KNEE': None,
                    'L_KNEE': None,
                    'R_ANK_PITCH': None,
                    'L_ANK_PITCH': None,
                    'R_ANK_ROLL': None,
                    'L_ANK_ROLL': None,
                    'HEAD_PAN': None,
                    'HEAD_TILT': None
                     }

        self.timing = {'Time': None,
                    'PauseTime': None
                    }
        self.title = "Poses2"

    def loadPage(self, page):

        self.poses = page.find('Poses')
        self.poseclasses = [pc.findall('.//PoseClass') for pc in self.poses]
        # self.poses = page.find('Poses')
        self.title = page.find('Title').text

        # self.allpages = []

        self.extractPoses()

    def extractPoses(self):
#         self.pose_classes = [pc for pc in self.poses]
#         print self.poses[0]
#         print self.pose_classes
#         for pose_class in self.poses:
#             print pose_class

        # fbody = defaultdict(list)
        for key in self.body.keys():
            self.body[key] = self._getJointPoses(key, self.poses)

        for key in self.timing.keys():
            self.timing[key] = self._getJointPoses(key, self.poses)

        # return allpages

    def loadPoses(self, sequence):
        rospy.loginfo("[loadPoses] Sequence: %s" % type(sequence))
        for key, value in sequence.iteritems():
            if key in self.body.keys():
                self.body[key] = value
            elif key in self.timing.keys():
                self.timing[key] = value

    def _getJointPoses(self, joint_name, poses):
        return [int(pcx.find(joint_name).text) for pcx in poses]

    def setTitle(self, title):
        self.title = title

    def getTitle(self):
        return self.title

    def getPoses(self):
        return self.body

    def getTiming(self):
        return self.timing

    def addInitialPose(self, init_pose, init_timing=None):
        init_body = {'R_SHO_PITCH': 512,
                    'L_SHO_PITCH': 512,
                    'R_SHO_ROLL': 512,
                    'L_SHO_ROLL': 512,
                    'R_ELBOW_ROLL': 512,
                    'R_ELBOW_PITCH': 512,
                    'L_ELBOW_ROLL': 512,
                    'L_ELBOW_PITCH': 512,
                    'R_HIP_YAW': 512,
                    'L_HIP_YAW': 512,
                    'R_HIP_ROLL': 512,
                    'L_HIP_ROLL': 512,
                    'R_HIP_PITCH': 512,
                    'L_HIP_PITCH': 512,
                    'R_KNEE': 512,
                    'L_KNEE': 512,
                    'R_ANK_PITCH': 512,
                    'L_ANK_PITCH': 512,
                    'R_ANK_ROLL': 512,
                    'L_ANK_ROLL': 512,
                    'HEAD_PAN': 512,
                    'HEAD_TILT': 512
                     }
        i_timing = {'Time': 550,
                    'PauseTime': 100
                    }
        try:
            for key, value in init_body.iteritems():
                self.body[key].insert(0, value)
        except AttributeError as ae:
            rospy.loginfo('WAT: {}'.format(init_body))

        for key, value in i_timing.iteritems():
            self.timing[key].insert(0, value)
            self.timing[key].insert(0, value)

        for key, value in init_pose.iteritems():
            self.body[key] = [value]*2 + self.body[key][1:]

        if init_timing != None:
            for key, value in init_timing.iteritems():
                self.timing[key][0] = [value]*2 + self.timing[key][2:]

class JeevesHeadPoses(BasePoses):
    def __init__(self):
        self.body = {'TILT_RIGHT': None,
                    'TILT_LEFT': None,
                    'PAN': None,
                    'FACE_CMD': None
                     }

        super(BasePoses, self).__init__()
