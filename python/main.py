import time
import numpy as np
from dynamixelArm import RobotArm  

def main():
    arm = RobotArm() 
    sampling_period = 1  # Frequency in seconds (1Hz = 1 second between updates)
    lastTime = 0
    startTime = time.time()
    trackingPoints = np.array([])

    try:
        while True:        
            if (time.time() - lastTime > sampling_period):
                # move robot arm to next point
                angle = 100*np.sin(2*np.pi*0.5*(time.time()-startTime))+512
                arm.set_goal_position(2,angle) # test with 1 motor for now
                lastTime = time.time()
    finally:
        # Ensure proper cleanup
        arm.close()
        
            

if __name__ == "__main__":
    main()
