import time
import numpy as np
from dynamixelArm import RobotArm 

def main():
    arm = RobotArm(device_name="/dev/ttyACM0",end_effector="straight")

    

    t1 = arm.task_polyTraj(A={'gamma': 0, 'origin': [0,0.15,0.1], 'elbow':"up", 'v': 0, 'a': 0},
                          B={'gamma': -np.pi/4, 'origin': [0,0.15,0.1], 'elbow':"up", 'v': 0, 'a': 0},
                          T = 3,
                          order = 3)
    t2 = arm.task_polyTraj(A={'gamma': -np.pi/4, 'origin': [0,0.15,0.1], 'elbow':"up", 'v': 0, 'a': 0},
                          B={'gamma': 0, 'origin': [0,0.15,0.1], 'elbow':"up", 'v': 0, 'a': 0},
                          T = 3,
                          order = 3)
    t1 = arm.task_polyTraj(A={'gamma': 0, 'origin': [0,0.1,0.1], 'elbow':"up", 'v': 0, 'a': 0},
                          B={'gamma': -np.pi/1, 'origin': [0,0.1,0.1], 'elbow':"up", 'v': 0, 'a': 0},
                          T = 3,
                          order = 3)
    t2 = arm.task_polyTraj(A={'gamma': -np.pi/1, 'origin': [0,0.1,0.1], 'elbow':"up", 'v': 0, 'a': 0},
                          B={'gamma': 0, 'origin': [0,0.1,0.1], 'elbow':"up", 'v': 0, 'a': 0},
                          T = 3,
                          order = 3)
    
    
    traj = [t1,t2]
    i = 0

    # t1.plot()
    

    try:
        while(True):

            thread, DONE = arm.run_in_thread(arm.follow_traj,traj[i],Ts=0.1,frame_no=5,elbow="up")
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
    
