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

    def circle(self,center: list,radius = 0.03, gamma = -np.pi/2,frame_no = 4):
        
        _,o_0,gamma_0 = self.arm.fwd_kin(frame_no=frame_no,return_details=True)
            
        func = [{'type':"sin",'amplitude':radius,'phase':0,'freq':2*np.pi,'offset':center[0]},
                {'type':"cos",'amplitude':radius,'phase':0,'freq':2*np.pi,'offset':center[1]},
                {'type':"const",'a0':center[2]},
                {'type':"const",'a0':gamma}]
        A = {'v':0,'a':0}
        B = {'v':0,'a':0}
        circle_traj = self.arm.task_genericTraj(func,A=A,B=B,T=2,order=3)

        circle = []
        circle.append(self.arm.joint_polyTraj(frame_no=frame_no, 
                            A={'gamma': gamma_0, 'origin': o_0, 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                            B={'gamma': gamma, 'origin': circle_traj.s_0[0:3], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                            tA = 0,
                            tB = 3,
                            order = 3))
        circle.append(circle_traj)
        circle.append(self.arm.joint_polyTraj(frame_no=frame_no, 
                            A={'gamma': gamma, 'origin': circle_traj.s_0[0:3], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                            B={'gamma': gamma_0, 'origin': o_0, 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                            tA = 0,
                            tB = 3,
                            order = 3))

        # circle.append(self.arm.task_polyTraj(A={'gamma': -np.pi/2, 'origin': [center[0]+size/2,center[1]+size/2,center[2]], 'elbow':"up", 'v': 0, 'a': 0},
        #                     B={'gamma': gamma, 'origin': [center[0]-size/2,center[1]-size/2,center[2]], 'elbow':"up", 'v': 0, 'a': 0},
        #                     T = 1,
        #                     order = 3))
        # circle.append(self.arm.joint_polyTraj(frame_no=frame_no, 
        #                     A={'gamma': gamma, 'origin': [center[0]+size/2,center[1]-size/2,center[2]], 'elbow':"up", 'v': [0,0,0.05], 'gamma_d': 0},
        #                     B={'gamma': gamma_0, 'origin': o_0, 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
        #                     tA = 0,
        #                     tB = 3,
        #                     order = 3))

        thread, DONE = self.arm.run_in_thread(self.arm.follow_traj,circle,Ts=0.1,frame_no=frame_no,elbow="up")

        return thread, DONE                 
