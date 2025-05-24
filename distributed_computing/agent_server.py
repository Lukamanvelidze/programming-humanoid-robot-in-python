'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import xmlrpc.server
import threading
import xmlrpc
from xmlrpc.server import SimpleXMLRPCServer
import os
import sys
import time
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.target_joints[joint_name]
        # YOUR CODE HERE
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes  # set once, triggering interpolation somewhere else
        self.cycle_completed = False
        while not self.cycle_completed:
            print(self.cycle_completed)
            
        # Wait for cycle to complete, but allow other processes to run
            time.sleep(0.05)  # 10ms sleep to yield CPU
        



    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name]
       

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(effector_name,transform)

if __name__ == '__main__':
    server = SimpleXMLRPCServer(("localhost", 8000), allow_none=True)

    agent = ServerAgent()
    server.register_introspection_functions()
    server.register_instance(agent)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    agent.run()

