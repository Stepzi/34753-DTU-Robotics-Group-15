import time
import numpy as np
from dynamixelArm import RobotArm 

def main():
    arm = RobotArm(device_name="/dev/ttyACM0",end_effector="straight")
    sampling_period = 1  # Frequency in seconds (1Hz = 1 second between updates)
    last_time =  0
    start_time = time.time()
    
    

    t1 = arm.joint_polyTraj(frame_no=5, 
                          A={'gamma': 0, 'origin': [0,0.15,0.15], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          B={'gamma': np.deg2rad(90), 'origin': [0,0.1,0.2], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          tA = 0,
                          tB = 3,
                          order = 1)
    t2 = arm.joint_polyTraj(frame_no=5, 
                          A={'gamma': np.deg2rad(90), 'origin': [0,0.1,0.2], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          B={'gamma': 0, 'origin': [0,0.15,0.15], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          tA = 0,
                          tB = 5,
                          order = 5)
    t3 = arm.joint_polyTraj(frame_no=5, 
                          A={'gamma': 0, 'origin': [0,0.15,0.15], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          B={'gamma': 0, 'origin': [0,0.15,0.05], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          tA = 0,
                          tB = 5,
                          order = 5)
    t4 = arm.joint_polyTraj(frame_no=5, 
                          A={'gamma': 0, 'origin': [0,0.15,0.05], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          B={'gamma': 0, 'origin': [0,0.15,0.15], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          tA = 0,
                          tB = 5,
                          order = 5)
    
    traj = [t1,t2,t3,t4]
    i = 0

    # t1.plot()
    

    try:
        while(True):

            thread, DONE = arm.run_in_thread(arm.follow_traj,traj[i],Ts=0.1)
            while(not DONE.is_set()):
                arm.twin.draw_arm(draw_jointSpace=True)
                time.sleep(0.005)
        
            i = (i+1)%len(traj)
            
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
    
