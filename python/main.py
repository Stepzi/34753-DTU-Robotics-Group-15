import time
import numpy as np
from dynamixelArm import RobotArm 
from digitalTwin import DigitalTwin 

def main():
    arm = RobotArm(device_name="dev/ttyACM0")
    sampling_period = 0.1  # Frequency in seconds (1Hz = 1 second between updates)
    lastTime = 0
    startTime = time.time()
    trackingPoints = np.array([])

    try:
        while True:        
            if (time.time() - lastTime > sampling_period):
                # move robot arm to next point
                angle_deg = 30*np.sin(2*np.pi*0.5*(time.time()-startTime))
                arm.set_joint_angle(2,np.deg2rad(angle_deg)) # test with 1 motor for now
                lastTime = time.time()
            arm.get_joint_angle()                
            arm.twin.draw_arm()

    
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        # Ensure proper cleanup
        arm.close()
        
            

if __name__ == "__main__":
    main()
    
