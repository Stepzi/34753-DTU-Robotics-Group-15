import time
import numpy as np
from dynamixelArm import RobotArm 

def main():
    arm = RobotArm(device_name="/dev/ttyACM0",end_effector="straight")
    sampling_period = 5  # Frequency in seconds (1Hz = 1 second between updates)
    lastTime = time.time()
    startTime = time.time()
    
    i = 0

    points = [[0,0.15,0.15],
              [0,0.15,0.0]]
    gammas = [0,
              0]

    try:
        while True:        
            if (time.time() - lastTime > sampling_period):
                lastTime = time.time()
                
                # tool = arm.Frames[5].T_local()
                tool = None
                try:
                    q = arm.inv_kin(gammas[i],points[i],elbow="up",tool=tool)
                except Exception as e: print(e)
                else:
                    arm.move_to_angles(q)
                
                i = (i+1)%len(points)
            
            arm.twin.draw_arm()
                        
            time.sleep(0.005)
            # inp = input()

    
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        # Ensure proper cleanup
        arm.close()
        
            

if __name__ == "__main__":
    main()
    
