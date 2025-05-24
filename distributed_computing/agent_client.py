'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import threading
import xmlrpc.client
import numpy as np
from keyframes import hello, rightBackToStand

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.execute_keyframes, args=[keyframes])
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.set_transform, args=[effector_name, transform])
        thread.start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.rpcserver= xmlrpc.client.ServerProxy("http://localhost:8000")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.rpcserver.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.rpcserver.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        self.rpcserver.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.rpcserver.execute_keyframes(keyframes)
        
    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        m = self.rpcserver.get_transform(name)
        return np.array(m).reshape((4, 4), order='F')

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        
        # YOUR CODE HERE
        matrix_values = transform.reshape(-1, order='F').astype(float).tolist()
        self.rpcserver.set_transform(effector_name, matrix_values)


if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    joint = 'LKneePitch'  # Replace with anything else if u want to test, this is veeeery simple and basic, but well it can check your code partially.
    agent.execute_keyframes(hello())
    agent.set_angle(joint, 0)
    print("New angle:", agent.get_angle(joint))


