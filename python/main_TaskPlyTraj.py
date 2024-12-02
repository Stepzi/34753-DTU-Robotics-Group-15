import time
import numpy as np
from dynamixelArm import RobotArm 

def main():
    arm = RobotArm(device_name="/dev/ttyACM00",end_effector="straight")
    startTime = time.time()
    lastTime = 0


    func = [{'type':"sin",'amplitude':np.pi/2,'phase':0,'freq':2*np.pi,'offset':0}]
    A = {'v':0,'a':0}
    B = {'v':0,'a':0}
    q = arm.task_genericTraj(func,A=A,B=B,T=10,order=3)

    try:
        while(True):

            if (time.time() - lastTime > 0.1):
                lastTime = time.time()
                t = time.time() - startTime

                arm.set_joint_angle(4,q.eval(t)[0])
                arm.twin.draw_arm(draw_jointSpace=True)
        
            if (t > 10):
                input()
                         
            time.sleep(0.005)
            

    
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        # Ensure proper cleanup
        arm.close()
        
            

if __name__ == "__main__":
    main()
    
