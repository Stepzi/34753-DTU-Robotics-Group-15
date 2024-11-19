import sys, os
import numpy as np

# Add the Dynamixel SDK path to sys.path
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'DynamixelSDK/python/src')))

import dynamixel_sdk as dxl

class RobotArm():
    def __init__(self, device_name='COM5', baudrate=1000000, protocol_version=1.0):
        # initialize constants and configurations
        self.__ADDR_MX_TORQUE_ENABLE = 24
        self.__ADDR_MX_CW_COMPLIANCE_MARGIN = 26
        self.__ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
        self.__ADDR_MX_CW_COMPLIANCE_SLOPE = 28
        self.__ADDR_MX_CCW_COMPLIANCE_SLOPE = 29
        self.__ADDR_MX_GOAL_POSITION = 30
        self.__ADDR_MX_MOVING_SPEED = 32
        self.__ADDR_MX_PRESENT_POSITION = 36
        self.__ADDR_MX_PUNCH = 48
        self.__TORQUE_ENABLE = 1
        self.__TORQUE_DISABLE = 0
        self.__DXL_MOVING_STATUS_THRESHOLD = 10  # Threshold for detecting movement completion
        # self.__DXL_IDS = [1, 2, 3, 4]  # Motor IDs
        self.__DXL_IDS = [2, 3, 4] # comment out motor 1 (robot 10)


        # Robot Configuration
        self.__SV_joint_angles = np.array([512,512,512,512])
        self.__PV_joint_angles = np.array([0,0,0,0])
        self.__motor_speed = [0.1,0.1,0.1,0.1] 
       

        # Build Kinematic Chain
        self._Links = self.assemble_robot()


        # initialize __portHandler and __packetHandler
        self.__portHandler = dxl.PortHandler(device_name)
        self.__packetHandler = dxl.PacketHandler(protocol_version)

        # open port and set baudrate
        try: 
            self.__portHandler.openPort()
            self.__portHandler.setBaudRate(baudrate)
            print(f"Successfully opened port {device_name} and set baudrate")
            self.__has_hardware = True
        except:
            print(f"Failed to open port {device_name} and set baudrate")
            self.__has_hardware = False
            # sys.exit(1)

        self.set_speed(self.__motor_speed,overwrite_speeds=False)

    def assemble_robot(self):
        links = [] # List of Link Objects
        links.append(Link(theta=0,d=0.05,a=0,alpha = 0))           # Base Frame
        links.append(Link(theta=0,d=0.05,a=0,alpha = np.pi/2))
        links.append(Link(theta=np.pi/2,d=0,a=0.093,alpha = 0))
        links.append(Link(theta=0,d=0,a=0.093,alpha = 0))
        links.append(Link(theta=0,d=0,a=0.05,alpha = 0))
        return links 

    # Kinematic Methods
    def fwd_kin(self):
        links = [] # List of Link Objects
        return links 

    # Create "functions" for setting and moving motors:
    def enable_torque(self, motor_id):
        # enable torque for a motor
        result, error = self.__packetHandler.write1ByteTxRx(self.__portHandler, motor_id, self.__ADDR_MX_TORQUE_ENABLE, self.__TORQUE_ENABLE)
        if result != dxl.COMM_SUCCESS:
            print(f"Failed to enable torque for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")

    def disable_torque(self, motor_id):
        # Disable torque for a specific motor
        result, error = self.__packetHandler.write1ByteTxRx(self.__portHandler, motor_id, self.__ADDR_MX_TORQUE_ENABLE, self.__TORQUE_DISABLE)
        if result != dxl.COMM_SUCCESS:
            print(f"Failed to disable torque for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")

    def deg_to_rot(self,deg):
        #deg_to_rot function for the robot arm class.
        #   Converts degree to units per rotation of motors
        #
        #Inputs:
        #   deg : value [deg]
        #Outputs:
        #   rot : value in units per rotation of motor
        return deg*1/0.293+512

    def rad_to_rot(self,rad):
        #rad_to_rot function for the robot arm class.
        #   Converts radians to units per rotation of motors
        #
        #Inputs:
        #   rad : value [rad]
        #Outputs:
        #   rot : value in units per rotation of motor
        return self.deg_to_rot(np.rad2deg(rad))

    def rot_to_deg(self,rot):
        #rot_to_deg function for the robot arm class.
        #   Convers units per rotation of motors to degree
        #
        #Inputs:
        #   rot : value in units per rotation of motor
        #Outputs:
        #   deg : value [deg]
        return (rot-512)*0.293

    def rot_to_rad(self,rot):
        #rot_to_rad function for the robot arm class.
        #   Convers units per rotation of motors to radians
        #
        #Inputs:
        #   rot : value in units per rotation of motor
        #Outputs:
        #   rad : value [rad]
        return np.deg2rad(self.rot_to_deg(rot))
    
    def set_joint_angle(self, joint, position):
        
        self.__SV_joint_angles[joint-1] = position

        if self.__has_hardware:
            # Set the target position for a motor
            result, error = self.__packetHandler.write2ByteTxRx(self.__portHandler, self.__DXL_IDS[joint-1], self.__ADDR_MX_GOAL_POSITION, position)
            if result != dxl.COMM_SUCCESS:
                print(f"Failed to set goal position for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")


    def get_joint_angle(self, joint=None):
        if self.__has_hardware:
            for i, motor_id in enumerate(self.__DXL_IDS):                 
                    # Attempt to read the present position for the specified motor
                    self.__PV_joint_angles[i], result, error = self.__packetHandler.read2ByteTxRx(self.__portHandler, motor_id, self.__ADDR_MX_PRESENT_POSITION)
                
                    if result != dxl.COMM_SUCCESS:
                        print(f"Failed to read position for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")
                        return None
                    elif error != 0:
                        print(f"Error occurred while reading position for motor {motor_id}: {self.__packetHandler.getRxPacketError(error)}")
                        return None
        else:
            self.__PV_joint_angles = self.__SV_joint_angles

        if joint == None:
            return self.__PV_joint_angles
        else:
            assert (joint >= 1 and joint <= 4), f"Joint {joint} does not exist"
            return self.__PV_joint_angles[joint-1]


    def move_to_angles(self, goal_positions):
        # Move each motor to the target position specified in goal_positions
        for motor_id, goal_pos in zip(self.__DXL_IDS, goal_positions):
            self.set_joint_angle(motor_id, goal_pos)

        # Wait until all motors reach their goal positions
        for motor_id, goal_pos in zip(self.__DXL_IDS, goal_positions):
            while True:
                current_position = self.get_joint_angle(motor_id)
                if current_position is None:
                    break  # Exit if we failed to read the position
                if abs(goal_pos - current_position) < self.__DXL_MOVING_STATUS_THRESHOLD:
                    break  # Movement complete for this motor

    #TODO: Fix this function

    # def set_torque(self, enable=True):
    #     for motor_id in self.__DXL_IDS:
    #         result, error = self.__packetHandler.write1ByteTxRx(self)
           

    def set_speed(self, speeds, overwrite_speeds):
        #set_speed function for the robot arm class.
        #   Sets individual motor speeds between 0# and 100#
        #
        #Inputs:
        #   speeds : a vector representing motor speeds for each motor
        #   ID between 0 and 1
        #   overwrite_speeds: boolean, if true class internal motor
        #   speeds are overwritten to motor speeds of function call
        #Outputs:
        #   None
        if overwrite_speeds:
            self.__motor_speed = speeds

        for motor_id, speed in zip(self.__DXL_IDS, speeds):
            assert (speed > 0 and speed <= 1),"Movement speed out of range, enter value between ]0,1]"
            if self.__has_hardware:
                result, error = self.__packetHandler.write2ByteTxRx(self.__portHandler,motor_id, self.__ADDR_MX_MOVING_SPEED, int(speed*1023))
                if result != dxl.COMM_SUCCESS:
                    print(f"Failed to set speed for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")
    
            
    

    def close(self):
        # Disable torque and close port before exiting
        if self.__has_hardware:
            for motor_id in self.__DXL_IDS:
                self.disable_torque(motor_id)
            self.__portHandler.closePort()

class Link():
    def __init__(self,theta,d,a,alpha,type="revolute"):

        self.__revolute = False
        self.__theta = theta # joint angle is offset by this value (relevant for frame 2)
        self.__d = d
        self.__a = a
        self.__alpha = alpha
                
        if type == "revolute":
            self.__revolute = True
        
    def T_local(self,q):
        if self.__revolute:
           return np.array([[np.cos(q+self.__theta), -np.sin(q+self.__theta)*np.cos(self.__alpha), np.sin(q+self.__theta)*np.sin(self.__alpha), self.__a*np.cos(q+self.__theta)],
                           [np.sin(q+self.__theta), np.cos(q+self.__theta)*np.cos(self.__alpha), -np.cos(q+self.__theta)*np.sin(self.__alpha), self.__a*np.sin(q+self.__theta)],
                           [0, np.sin(self.__alpha), np.cos(self.__alpha), self.__d],
                           [0, 0, 0, 1]]) 
        else:
           return np.array([[np.cos(self.__theta), -np.sin(self.__theta)*np.cos(self.__alpha), np.sin(self.__theta)*np.sin(self.__alpha), self.__a*np.cos(self.__theta)],
                           [np.sin(self.__theta), np.cos(self.__theta)*np.cos(self.__alpha), -np.cos(self.__theta)*np.sin(self.__alpha), self.__a*np.sin(self.__theta)],
                           [0, np.sin(self.__alpha), np.cos(self.__alpha), q],
                           [0, 0, 0, 1]]) 
    
    def T_global(self,parent,q):
        return parent @ self.T_local(q)



# basic usage demo:

if __name__ == "__main__":
    arm = RobotArm(device_name='/dev/ttyACM0', baudrate=1000000)

    try:
        #self.__packetHandler.write2ByteTxRx(self.__portHandler, self.DXL_ID, self.__ADDR_MX_MOVING_SPEED, 100)
        # enable torque on all motors
        for motor_id in arm.__DXL_IDS:
            arm.enable_torque(motor_id)

        # Define speed for each motor
        arm.set_speed([0.01, 0.01, 0.01, 0.01],True)
            

        # Define goal positions for each motor
        goal_positions = [512, 512, 512, 512] # 512 is the "zero" position. 512 at each motor will make the arm straight
        # dont write to motor 1?
        arm.move_to_angles(goal_positions)
        
    finally:
        # Ensure proper cleanup
        arm.close()







               # Enable torque and configure other settings for each Dynamixel
                #for DXL_ID in __DXL_IDS:
                #    __packetHandler.write1ByteTxRx(__portHandler, DXL_ID, __ADDR_MX_TORQUE_ENABLE, __TORQUE_ENABLE)
                #    __packetHandler.write2ByteTxRx(__portHandler, DXL_ID, __ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
                #    __packetHandler.write2ByteTxRx(__portHandler, DXL_ID, __ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
                #    __packetHandler.write1ByteTxRx(__portHandler, DXL_ID, __ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
                #    __packetHandler.write1ByteTxRx(__portHandler, DXL_ID, __ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
                #    __packetHandler.write2ByteTxRx(__portHandler, DXL_ID, __ADDR_MX_MOVING_SPEED, 100)
                    #__packetHandler.write2ByteTxRx(__portHandler, DXL_ID, __ADDR_MX_GOAL_POSITION, 512) # function to move arm
                    # instead of these functions we should be able to use the new ones i created below


# how to include kinematics

# try to copy the move_j function, use degree conversion like in myrobot
# if that doesnt work, try to use chatgpt's method of mapping degrees
# copy the function that checks if angle is within motor limits 

