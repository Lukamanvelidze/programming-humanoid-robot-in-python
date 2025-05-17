'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np




class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        self.ranges = {'HeadYaw': (-2.0857, 2.0857),
                       'HeadPitch': (-0.6720, 0.5149),
                       'LShoulderPitch': (-2.0857, 2.0857),
                       'RShoulderPitch': (-2.0857, 2.0857),
                       'LShoulderRoll': (-0.3142, 1.3265),
                       'RShoulderRoll': (-0.3142, 1.3265),
                       'LElbowYaw': (-2.0857, 2.0857),
                       'RElbowYaw': (-2.0857, 2.0857),
                       'LElbowRoll': (-1.5446, 0.0349),
                       'RElbowRoll': (-1.5446, 0.0349),
                       'LHipYawPitch': (-1.145303, 0.740810),
                       'RHipYawPitch': (-1.145303, 0.740810),
                       'LHipRoll': (-0.379472, 0.790477),
                       'RHipRoll': (-0.379472, 0.790477),
                       'LHipPitch': (-1.535889, 0.484090),
                       'RHipPitch': (-1.535889, 0.484090),
                       'LKneePitch': (-0.092346, 2.112528),
                       'RKneePitch': (-0.092346, 2.112528),
                       'LAnklePitch': (-1.189516, 0.922747),
                       'RAnklePitch': (-1.189516, 0.922747),
                       'LAnkleRoll': (-0.397880, 0.769001),
                       'RAnkleRoll': (-0.397880, 0.769001),}
         # Extract position vector p and rotation matrix R from transform
        p = np.asarray(transform[:3, 3]).flatten()   # ensure p is 1D array shape (3,)
        R = np.asarray(transform[:3, :3])           
    
        # NAO robot link lengths in meters
        HipOffsetZ = 0.085
        HipOffsetY = 0.050
        ThighLength = 0.100
        TibiaLength = 0.1029
        FootHeight = 0.04519
    
        # Adjust position for foot height to get ankle position
        ankle_pos = p.copy()
        ankle_pos[2] += FootHeight
    
        # Sign for left/right leg
        sign = 1 if effector_name == 'LLeg' else -1
    
        # Hip position in torso frame
        hip_pos = np.array([0, sign * HipOffsetY, -HipOffsetZ])
    
        # Vector from hip to ankle
        vec = ankle_pos - hip_pos
    
        # --- HipYawPitch angle ---
        hip_yaw_pitch = np.arctan2(vec[1], vec[0])
    
        # Rotation matrix for rotating vec by -hip_yaw_pitch around z-axis
        c, s = np.cos(-hip_yaw_pitch), np.sin(-hip_yaw_pitch)
        rot_z = np.array([[c, -s, 0],
                          [s,  c, 0],
                          [0,  0, 1]])
    
        # Rotate vec into sagittal plane
        vec_sag = rot_z @ vec
    
        # Hip roll angle
        hip_roll = np.arctan2(vec_sag[1], vec_sag[2])
    
        # Distance from hip to ankle projected on sagittal plane (x-z)
        L = np.linalg.norm(vec_sag[[0, 2]])
    
        # Law of cosines for knee pitch
        cos_knee = (ThighLength**2 + TibiaLength**2 - L**2) / (2 * ThighLength * TibiaLength)
        cos_knee = np.clip(cos_knee, -1.0, 1.0)
        knee_pitch = np.pi - np.arccos(cos_knee)
    
        # Angle alpha for hip pitch calculation
        cos_alpha = (ThighLength**2 + L**2 - TibiaLength**2) / (2 * ThighLength * L)
        cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
        alpha = np.arccos(cos_alpha)
    
        # Hip pitch angle
        hip_pitch = -(np.arctan2(-vec_sag[2], vec_sag[0]) + alpha)
    
        # Ankle pitch angle
        ankle_pitch = -(hip_pitch + knee_pitch)
    
        # Ankle roll angle calculated from rotation matrix minus hip roll
        ankle_roll = np.arctan2(R[2, 1], R[2, 2]) - hip_roll
    
        # Collect all joint angles
        joint_angles = [hip_yaw_pitch, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll]
        clamped_angles = []
        for i, angle in enumerate(joint_angles):
            joint_name = self.chains[effector_name][i]
            min_angle, max_angle = self.ranges[joint_name]
            clamped_angle = np.clip(angle, min_angle, max_angle)
            clamped_angles.append(clamped_angle)
    
        return clamped_angles
        


    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)

    
        self.keyframes = ([], [], [])
        for i in range(0, len(joint_angles)):
            joint_name = self.chains[effector_name][i]
            self.keyframes[0].append(joint_name)
            self.keyframes[1].append([5, 10])  # adjust timing if needed
            self.keyframes[2].append([
                [joint_angles[i], [3, 0, 0], [3, 0, 0]],
                [0.0,        [3, 0, 0], [3, 0, 0]]
            ])
            

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    T[-1, 0] = 0.2
    print("Before set_transforms")
    agent.set_transforms('LLeg', T)
    print("AFTER set_transforms")
    agent.run()
