'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import rightBellyToStand
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.cycle_completed = False
        self._last_cycle_time = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        #target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        names, all_times, all_keys = keyframes
        current_time = perception.time

        for joint_idx, joint_name in enumerate(names):
            if joint_idx >= len(all_times) or joint_idx >= len(all_keys):
                continue

            times = all_times[joint_idx]
            keys = all_keys[joint_idx]

            if not times or not keys:
                continue

            angles = [key[0] for key in keys]
            max_time = max(times) if times else 1.0
            if self._last_cycle_time is None:
                self._last_cycle_time = current_time

                
            cycle_time = current_time % max_time

            if (self._last_cycle_time % max_time) > cycle_time:
                self.cycle_completed = True

            self._last_cycle_time = current_time

            if self.cycle_completed:
        
                for joint_idx, joint_name in enumerate(names):
                    angles = [key[0] for key in all_keys[joint_idx]]
                    target_joints[joint_name] = angles[-1]
                return target_joints
            
            else :
                # Find interval
                    for i in range(len(times) - 1):
                        if times[i] <= cycle_time <= times[i + 1]:
                            #  cubic spline calculation
                            t = (cycle_time - times[i]) / (times[i + 1] - times[i])
        
                            # Calculate derivatives
                            if i == 0:
                                m0 = (angles[1] - angles[0]) / (times[1] - times[0])
                            else:
                                m0 = (angles[i + 1] - angles[i - 1]) / (times[i + 1] - times[i - 1])
        
                            if i == len(times) - 2:
                                m1 = (angles[-1] - angles[-2]) / (times[-1] - times[-2])
                            else:
                                m1 = (angles[i + 2] - angles[i]) / (times[i + 2] - times[i])
        
                            # Hermite interpolation
                            h00 = 2 * t ** 3 - 3 * t ** 2 + 1
                            h10 = t ** 3 - 2 * t ** 2 + t
                            h01 = -2 * t ** 3 + 3 * t ** 2
                            h11 = t ** 3 - t ** 2
        
                            angle = (h00 * angles[i] +
                                     h10 * (times[i + 1] - times[i]) * m0 +
                                     h01 * angles[i + 1] +
                                     h11 * (times[i + 1] - times[i]) * m1)
        
                            target_joints[joint_name] = angle
                            break
                        else:  # No interval found (before first or after last)
                            target_joints[joint_name] = angles[0] if cycle_time < times[0] else angles[-1]

        # Handle mirrored joint, for some reason when i use the provided code line 39 causes an error, so i try to handle that case here...
        target_joints.setdefault('RHipYawPitch', target_joints.get('LHipYawPitch', 0))

        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello() # CHANGE DIFFERENT KEYFRAMES
    agent.run()
