import numpy as np

class Patterns():
    def __init__(self,rbt):

        self.arm = rbt

    def cross(self,center: list,size = 0.03, gamma = -np.pi/2,frame_no = 4):
        
        _,o_0,gamma_0 = self.arm.fwd_kin(frame_no=frame_no,return_details=True)

        cross = []
        cross.append(self.arm.joint_polyTraj(frame_no=frame_no, 
                            A={'gamma': gamma_0, 'origin': o_0, 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                            B={'gamma': gamma, 'origin': [center[0]+size/2,center[1]+size/2,center[2]], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                            tA = 0,
                            tB = 3,
                            order = 3))
        cross.append(self.arm.task_polyTraj(A={'gamma': -np.pi/2, 'origin': [center[0]+size/2,center[1]+size/2,center[2]], 'elbow':"up", 'v': 0, 'a': 0},
                            B={'gamma': gamma, 'origin': [center[0]-size/2,center[1]-size/2,center[2]], 'elbow':"up", 'v': 0, 'a': 0},
                            T = 1,
                            order = 3))
        cross.append(self.arm.joint_polyTraj(frame_no=frame_no, 
                            A={'gamma': gamma, 'origin': [center[0]-size/2,center[1]-size/2,center[2]], 'elbow':"up", 'v': [0,0,0.05], 'gamma_d': 0},
                            B={'gamma': gamma, 'origin': [center[0]-size/2,center[1]+size/2,center[2]], 'elbow':"up", 'v': [0,0,-0.05], 'gamma_d': 0},
                            tA = 0,
                            tB = 1,
                            order = 3))
        cross.append(self.arm.task_polyTraj(A={'gamma': gamma, 'origin': [center[0]-size/2,center[1]+size/2,center[2]], 'elbow':"up", 'v': 0, 'a': 0},
                            B={'gamma': gamma, 'origin': [center[0]+size/2,center[1]-size/2,center[2]], 'elbow':"up", 'v': 0, 'a': 0},
                            T = 1,
                            order = 3))
        cross.append(self.arm.joint_polyTraj(frame_no=frame_no, 
                            A={'gamma': gamma, 'origin': [center[0]+size/2,center[1]-size/2,center[2]], 'elbow':"up", 'v': [0,0,0.05], 'gamma_d': 0},
                            B={'gamma': gamma_0, 'origin': o_0, 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                            tA = 0,
                            tB = 3,
                            order = 3))

        thread, DONE = self.arm.run_in_thread(self.arm.follow_traj,cross,Ts=0.1,frame_no=frame_no,elbow="up")

        return thread, DONE                 
