import time
import numpy as np
from dynamixelArm import RobotArm 

def main():
    arm = RobotArm(device_name="/dev/ttyACM00",end_effector="straight")
        

    try:
        while(True):

            thread, DONE = arm.patterns.circle(center=[0.1,0.1,0.005],frame_no=5)
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
    
