import time
import numpy as np
from dynamixelArm import RobotArm 

def main():
    arm = RobotArm(device_name="/dev/ttyACM0",end_effector="straight")
    # arm.set_speed([0.1]*4)
    # arm.set_joint_angle(4,np.pi/2)
    # while True:
    #     _,o_0,gamma_0 = arm.fwd_kin(frame_no=4,return_details=True)
    #     print(np.rad2deg(gamma_0))
    #     time.sleep(0.1)

    # t1 = arm.task_polyTraj(A={'gamma': -np.pi/2, 'origin': [0,0.1,0.1], 'elbow':"up", 'v': 0, 'a': 0},
    #                       B={'gamma':  -np.pi/2, 'origin': [0,0.15,0.1], 'elbow':"up", 'v': 0, 'a': 0},
    #                       T = 1,
    #                       order = 3)
    # t2 = arm.task_polyTraj(A={'gamma':  -np.pi/2, 'origin': [0,0.15,0.1], 'elbow':"up", 'v': 0, 'a': 0},
    #                       B={'gamma': -np.pi/2, 'origin': [0,0.1,0.1], 'elbow':"up", 'v': 0, 'a': 0},
    #                       T = 1,
    #                       order = 3)

    z_height = 0.0
    cross = []
    cross.append(arm.joint_polyTraj(frame_no=4, 
                          A={'gamma': np.deg2rad(90), 'origin': [0,0.0,0.3255], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          B={'gamma': -np.deg2rad(90), 'origin': [0.02,0.15,z_height], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          tA = 0,
                          tB = 3,
                          order = 3))
    cross.append(arm.task_polyTraj(A={'gamma': -np.pi/2, 'origin': [0.02,0.15,z_height], 'elbow':"up", 'v': 0, 'a': 0},
                          B={'gamma': -np.pi/2, 'origin': [-0.02,0.11,z_height], 'elbow':"up", 'v': 0, 'a': 0},
                          T = 1,
                          order = 3))
    cross.append(arm.joint_polyTraj(frame_no=4, 
                          A={'gamma': -np.pi/2, 'origin': [-0.02,0.11,z_height], 'elbow':"up", 'v': [0,0,0.05], 'gamma_d': 0},
                          B={'gamma': -np.deg2rad(90), 'origin': [-0.02,0.15,z_height], 'elbow':"up", 'v': [0,0,-0.05], 'gamma_d': 0},
                          tA = 0,
                          tB = 1,
                          order = 3))
    cross.append(arm.task_polyTraj(A={'gamma': -np.pi/2, 'origin': [-0.02,0.15,z_height], 'elbow':"up", 'v': 0, 'a': 0},
                          B={'gamma': -np.pi/2, 'origin': [0.02,0.11,z_height], 'elbow':"up", 'v': 0, 'a': 0},
                          T = 1,
                          order = 3))
    
    # cross.append(arm.task_polyTraj(A={'gamma': -np.pi/2, 'origin': [0,0.1,0.0], 'elbow':"up", 'v': 0, 'a': 0},
    #                       B={'gamma': -np.pi/2, 'origin': [0,0.1,0.0], 'elbow':"up", 'v': 0, 'a': 0},
    #                       T = 3,
    #                       order = 3))
    
    

    # t1.plot()
    

    try:
        while(True):

            thread, DONE = arm.run_in_thread(arm.follow_traj,cross,Ts=0.1,frame_no=4,elbow="up")
            while(not DONE.is_set()):
                arm.twin.draw_arm(draw_jointSpace=True)
                time.sleep(0.005)
        
            
            print("hit enter for next move")
            inp = input()
                         
            time.sleep(0.005)
            

    
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        # Ensure proper cleanup
        arm.close()
        
            

if __name__ == "__main__":
    main()
    
