'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
import numpy as np

from numpy.matlib import matrix, identity

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {
            'Head': ['HeadYaw', 'HeadPitch'],
    
            'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
    
            'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
    
            'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
    
            'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
        }


    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)


    def rotation(self,axis, angle):
        c, s = np.cos(angle), np.sin(angle)
        R = np.eye(4)
        if axis == 'x':
            R[1:3, 1:3] = [[c, -s], [s, c]]
        elif axis == 'y':
            R[0:3:2, 0:3:2] = [[c, s], [-s, c]]
        elif axis == 'z':
            R[0:2, 0:2] = [[c, -s], [s, c]]
        return R

    def trans(self,x=0, y=0, z=0):
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        return T

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        # Parent-to-child link translation vectors in meters
        link_offsets = {
            # Head
            'HeadYaw':       [0.0, 0.0, 0.1265],       
            'HeadPitch':     [0.0, 0.0, 0.0],         
        
            # Left Arm
            'LShoulderPitch': [0.0, 0.098, 0.100],     
            'LShoulderRoll':  [0.105, 0.015, 0.0],     
            'LElbowYaw':      [0.0, 0.0, 0.0],         
            'LElbowRoll':     [0.05595, 0.0, 0.0],     
            'LWristYaw':      [0.0, 0.0, 0.0],         
        
            # Left Leg
            'LHipYawPitch':   [0.0, 0.050, -0.085],   
            'LHipRoll':       [0.0, 0.0, 0.0],         
            'LHipPitch':      [0.0, 0.0, 0.0],        
            'LKneePitch':     [0.0, 0.0, -0.100],     
            'LAnklePitch':    [0.0, 0.0, -0.1029],     
            'LAnkleRoll':     [0.0, 0.0, 0.0],         
        }
        trans_vec = link_offsets.get(joint_name, [0.0, 0.0, 0.0])
        lname = joint_name.lower()
        T = T @ self.trans(*trans_vec)
        if 'yaw' in lname:
            T = T @ self.rotation('z', joint_angle)
        elif 'pitch' in lname:
            T = T @ self.rotation('y', joint_angle)
        elif 'roll' in lname:
            T = T @ self.rotation('x', joint_angle)
        else:
        # No rotation or unknown joint type
            pass

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = Tl @ T

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
