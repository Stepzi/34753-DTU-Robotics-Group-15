import numpy as np

class Patterns():
    def __init__(self,rbt):

        self.arm = rbt

    def cross(self,center: list,size = 0.03, gamma = -np.pi/2,frame_no = 4,elbow="up",Ts = 0.1):
        
        _,o_0,gamma_0 = self.arm.fwd_kin(frame_no=frame_no,return_details=True)

        cross = []
        cross.append(self.arm.joint_polyTraj(frame_no=frame_no, 
                            A={'gamma': gamma_0, 'origin': o_0, 'elbow':elbow, 'v': [0,0,0], 'gamma_d': 0},
                            B={'gamma': gamma, 'origin': [center[0]+size/2,center[1]+size/2,center[2]], 'elbow':elbow, 'v': [0,0,0], 'gamma_d': 0},
                            tA = 0,
                            tB = 3,
                            order = 3))
        cross.append(self.arm.task_polyTraj(A={'gamma': gamma, 'origin': [center[0]+size/2,center[1]+size/2,center[2]], 'elbow':elbow, 'v': 0, 'a': 0},
                            B={'gamma': gamma, 'origin': [center[0]-size/2,center[1]-size/2,center[2]], 'elbow':elbow, 'v': 0, 'a': 0},
                            T = 1,
                            order = 3))
        cross.append(self.arm.joint_polyTraj(frame_no=frame_no, 
                            A={'gamma': gamma, 'origin': [center[0]-size/2,center[1]-size/2,center[2]], 'elbow':elbow, 'v': [0,0,0.05], 'gamma_d': 0},
                            B={'gamma': gamma, 'origin': [center[0]-size/2,center[1]+size/2,center[2]], 'elbow':elbow, 'v': [0,0,-0.05], 'gamma_d': 0},
                            tA = 0,
                            tB = 1,
                            order = 3))
        cross.append(self.arm.task_polyTraj(A={'gamma': gamma, 'origin': [center[0]-size/2,center[1]+size/2,center[2]], 'elbow':elbow, 'v': 0, 'a': 0},
                            B={'gamma': gamma, 'origin': [center[0]+size/2,center[1]-size/2,center[2]], 'elbow':elbow, 'v': 0, 'a': 0},
                            T = 1,
                            order = 3))
        cross.append(self.arm.joint_polyTraj(frame_no=frame_no, 
                            A={'gamma': gamma, 'origin': [center[0]+size/2,center[1]-size/2,center[2]], 'elbow':elbow, 'v': [0,0,0.05], 'gamma_d': 0},
                            B={'gamma': gamma_0, 'origin': o_0, 'elbow':elbow, 'v': [0,0,0], 'gamma_d': 0},
                            tA = 0,
                            tB = 3,
                            order = 3))

        thread, DONE = self.arm.run_in_thread(self.arm.follow_traj,cross,Ts=Ts,frame_no=frame_no,elbow=elbow)

        return thread, DONE 
                    
    def circle(self,center: list,radius = 0.02, gamma = -np.pi/2,frame_no = 4,elbow="up",Ts = 0.1):
        
        _,o_0,gamma_0 = self.arm.fwd_kin(frame_no=frame_no,return_details=True)

        circle = []
        circle.append(self.arm.joint_polyTraj(frame_no=frame_no, 
                            A={'gamma': gamma_0, 'origin': o_0, 'elbow':elbow, 'v': [0,0,0], 'gamma_d': 0},
                            B={'gamma': gamma, 'origin': [center[0],center[1]+radius,center[2]], 'elbow':elbow, 'v': [0,0,0], 'gamma_d': 0},
                            tA = 0,
                            tB = 3,
                            order = 3))
        
        t_total = 2
        segments = 10
        dphi = 2*np.pi/segments
        phis = np.linspace(dphi,2*np.pi,int(2*np.pi/dphi))
        for phi in phis: 
            print(np.rad2deg(phi))
            A={'gamma': gamma, 'origin': [center[0]+radius*np.sin(phi-dphi),center[1]+radius*np.cos(phi-dphi),center[2]], 'elbow':elbow, 'v': 0, 'a': 0}  
            B={'gamma': gamma, 'origin': [center[0]+radius*np.sin(phi),center[1]+radius*np.cos(phi),center[2]], 'elbow':elbow, 'v': 0, 'a': 0}    
            circle.append(self.arm.task_polyTraj(A=A,B=B,T = t_total/segments,order = 1))
            
        # circle.append(self.arm.joint_polyTraj(frame_no=frame_no, 
        #                     A={'gamma': B['gamma'], 'origin': B['origin'], 'elbow':elbow, 'v': [0,0,0], 'gamma_d': 0},
        #                     B={'gamma': gamma_0, 'origin': o_0, 'elbow':elbow, 'v': [0,0,0], 'gamma_d': 0},
        #                     tA = 0,
        #                     tB = 3,
        #                     order = 3))

        thread, DONE = self.arm.run_in_thread(self.arm.follow_traj,circle,Ts=Ts,frame_no=frame_no,elbow=elbow)

        return thread, DONE