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
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        names, times, keys = keyframes

        if self.start_time == None:
            self.start_time = self.perception.time

        current_time = self.perception.time - self.start_time

        #if start_time < 0:

        for i in range(len(names)):

            name = names[i]
            time = times[i]
            key = keys[i]

            for j in range(len(time) - 1):

                if j == 0 and current_time < time[0]:
                    t0 = 0
                    t3 = time[0]
                    p0 = np.array([t0,0])
                    p3 = np.array([t3,key[0][0]])

                elif j < len(time) - 1 and time[j] < current_time < time[j + 1]:
                    t0 = time[j]
                    t3 = time[j + 1]
                    p0 = np.array([t0,key[j][0]])
                    p3 = np.array([t3,key[j + 1][0]])

                else:
                    continue

                if j == 0:
                    t1 = 0 #versuch erstmal mit 0 und fuer p1 0,0
                    p1 = np.array([t1,0])

                else:
                    t1 = key[j][2][1] + t0
                    p1 = np.array([t1,key[j][2][2]]) + p0
                
                t2 = key[j + 1][1][1] + t3
                p2 = np.array([t2,key[j + 1][1][2]]) + p3

                t = (current_time - t0) / (t3 - t0)

                target_joints[name] = self.cubic_bezier(p0, p1, p2, p3, t)[1]

        return target_joints

    def cubic_bezier(self, p0, p1, p2, p3, t):
        return (1 - t)**3 * p0 + 3 * (1 - t)**2 * t * p1 + 3 * (t - 1)* t**2 * p2 + t**3 * p3

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()