import time
import numpy as np
from dynamixelArm import RobotArm 

def main():
    arm = RobotArm(device_name="dev/ttyACM0")
    sampling_period = 2  # Frequency in seconds (1Hz = 1 second between updates)
    lastTime = 0
    startTime = time.time()
    trackingPoints = np.array([])
    
    i = 0

    points = [[0,0.1,0],
              [0.15,0.0,0],
              [0,0.15,0.15]]
    gammas = [0,
              -np.pi/2,
              0]

    try:
        while True:        
            if (time.time() - lastTime > sampling_period):
                lastTime = time.time()
                # # move robot arm to next point
                # angle_deg = 30*np.sin(2*np.pi*0.5*(time.time()-startTime))
                # arm.set_joint_angle(2,np.deg2rad(angle_deg)) # test with 1 motor for now
                # arm.set_joint_angle(2,np.deg2rad(30)*i)
                
                
                try:
                    q = arm.inv_kin(gammas[i],points[i],"up")
                except Exception as e: print(e)
                else:
                    arm.move_to_angles(q)
                
                i = (i+1)%3

            arm.get_joint_angle()                
            arm.twin.draw_arm()
            
            time.sleep(0.005)

    
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        # Ensure proper cleanup
        arm.close()
        
            

if __name__ == "__main__":
    main()
    
