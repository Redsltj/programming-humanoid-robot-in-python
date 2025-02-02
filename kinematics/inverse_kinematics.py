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


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        
        # YOUR CODE HERE
        max_step = 0.1
        lambda_ = 1
        Ts = [identity(4)]
        
        for i in range(1000):
            for name in enumerate(self.chains[effector_name]):
                Ts.append(self.transforms[name])
        #unfinished
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        
        transform += self.transforms[self.chains[effector_name][-1]]            # transformation matrix 
        
        joint_angles = self.inverse_kinematics(effector_name, transform)        # return of the new angles after application of inverse kinematics to the chosen 
                                                                                # joint for the chosen transformation
        
        names = [effector_name]
        times = [[1.0]]*len(self.chains[effector_name])
        keys = []
        
        for i in range(len(joint_angles)):
            keys.append([joint_angles[i], [0, 0.0, 0.0]])                       # adding new angles to the keys matrix
            
        self.keyframes = (names, times, [keys])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
