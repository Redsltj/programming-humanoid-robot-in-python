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
import keyframes as keyframes
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        keys = keyframes[2]             #separating the elements of keyframes
        time = keyframes[1]
        name = keyframes[0]
        

        for i, joint in enumerate(name):
            for j in range(len(time[i])-1):
                self.start_time = time[j]
                currenttime = perception.time - self.start_time         
                
                if (time[i][j] < currenttime < time[i][len(time[i])-1]):    # duration of the movement
                    P_0 = keys[i][j][0]                                     #control points for the curve
                    P_0 = P_0 + keys[i][j][1][2]
                    P_3 = keys[i][j + 1][0]
                    P_2 = P_3 + keys[i][j][1][2]
                    t = (currenttime - time[i][j]) / (time[i][j + 1] - time[i][j])              #time of the movement
                    target_joints[joint] = (1-t)**3 * P_0 + 3*(1-t)**2 * t*P_1 + 3*(1-t) * t**2 * P_2 + t**3 * P_3      #bézier interpolation curve
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
