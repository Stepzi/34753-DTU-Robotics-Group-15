import time
import numpy as np
from dynamixelArm import RobotArm 

def main():
    arm = RobotArm(device_name="/dev/ttyACM0",end_effector="straight")
    sampling_period = 5  # Frequency in seconds (1Hz = 1 second between updates)
    lastTime = 0
    startTime = time.time()
    
    i = 0

    points = [[0,0.1,0.1],
              [0,0.1,0.1],
              [0,0.1,0.1],
              [0,0.1,0.1]]
    gammas = [0,
              -np.deg2rad(45),
              -np.deg2rad(90),
              -np.deg2rad(135)]

    try:
        while True:        
            if (time.time() - lastTime > sampling_period):
                lastTime = time.time()
                
                tool = arm.Frames[5].T_local()
                # tool = None
                try:
                    q = arm.inv_kin(gammas[i],points[i],elbow="up",tool=tool)
                except Exception as e: print(e)
                else:
                    arm.set_speed([2.0] *4)
                    arm.move_to_angles(q,blocking=False)
                
                i = (i+1)%len(points)
                # inp = input()
            
            arm.twin.draw_arm()
                        
            time.sleep(0.005)
            

    
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        # Ensure proper cleanup
        arm.close()
        
            

if __name__ == "__main__":
    main()
    
