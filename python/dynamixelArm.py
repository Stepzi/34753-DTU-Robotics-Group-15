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

        self.__motor_speed = 0 

        # initialize __portHandler and __packetHandler
        self.__portHandler = dxl.PortHandler(device_name)
        self.__packetHandler = dxl.PacketHandler(protocol_version)

        # open port and set baudrate
        if self.__portHandler.openPort() and self.__portHandler.setBaudRate(baudrate):
            print(f"Successfully opened port {device_name} and set baudrate")
        else:
            print(f"Failed to open port {device_name} and set baudrate")
            sys.exit(1)

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
            #deg_to_rot function for the MyRobot Class.
            #   Converts degree to units per rotation of motors
            #
            #Inputs:
            #   deg : value [deg]
            #Outputs:
            #   rot : value in units per rotation of motor
            return deg*1/0.293


    def rad_to_rot(self,rad):
            #rad_to_rot function for the MyRobot Class.
            #   Converts radians to units per rotation of motors
            #
            #Inputs:
            #   rad : value [rad]
            #Outputs:
            #   rot : value in units per rotation of motor
            return self.deg_to_rot(np.rad2deg(rad))

    def set_goal_position(self, motor_id, position):
        # Set the target position for a motor
        result, error = self.__packetHandler.write2ByteTxRx(self.__portHandler, motor_id, self.__ADDR_MX_GOAL_POSITION, position)
        if result != dxl.COMM_SUCCESS:
            print(f"Failed to set goal position for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")

    def get_present_position(self, motor_id):
        # Attempt to read the present position for the specified motor
        position, result, error = self.__packetHandler.read2ByteTxRx(self.__portHandler, motor_id, self.__ADDR_MX_PRESENT_POSITION)
    
        if result != dxl.COMM_SUCCESS:
            print(f"Failed to read position for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")
            return None
        elif error != 0:
            print(f"Error occurred while reading position for motor {motor_id}: {self.__packetHandler.getRxPacketError(error)}")
            return None

        # Position read successfully
        return position

    def move_to_positions(self, goal_positions):
        # Move each motor to the target position specified in goal_positions
        for motor_id, goal_pos in zip(self.__DXL_IDS, goal_positions):
            self.set_goal_position(motor_id, goal_pos)

        # Wait until all motors reach their goal positions
        for motor_id, goal_pos in zip(self.__DXL_IDS, goal_positions):
            while True:
                current_position = self.get_present_position(motor_id)
                if current_position is None:
                    break  # Exit if we failed to read the position
                if abs(goal_pos - current_position) < self.__DXL_MOVING_STATUS_THRESHOLD:
                    break  # Movement complete for this motor

    #TODO: Fix this function

    # def set_torque(self, enable=True):
    #     for motor_id in self.__DXL_IDS:
    #         result, error = self.__packetHandler.write1ByteTxRx(self)
           

    def set_speed(self, speeds, overwrite_speeds):
        #set_speed function for the MyRobot Class.
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
            if (speed > 0 and speed <= 1):
                result, error = self.__packetHandler.write2ByteTxRx(self.__portHandler,motor_id, self.__ADDR_MX_MOVING_SPEED, int(speed*1023))
                if result != dxl.COMM_SUCCESS:
                    print(f"Failed to set speed for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")
            else:
                print("\nMovement speed out of range, enter value between ]0,1]") 
            
    

    def close(self):
        # Disable torque and close port before exiting
        for motor_id in self.__DXL_IDS:
            self.disable_torque(motor_id)
        self.__portHandler.closePort()



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
        arm.move_to_positions(goal_positions)
        
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

