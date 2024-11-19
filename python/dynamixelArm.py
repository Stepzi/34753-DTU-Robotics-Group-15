import sys, os
import numpy as np

# Add the Dynamixel SDK path to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'DynamixelSDK/python/src')))

import dynamixel_sdk as dxl

class RobotArm():
    def __init__(self, device_name='COM5', baudrate=1000000, protocol_version=1.0):
        # Initialize constants and configurations
        self.ADDR_MX_TORQUE_ENABLE = 24
        self.ADDR_MX_CW_COMPLIANCE_MARGIN = 26
        self.ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
        self.ADDR_MX_CW_COMPLIANCE_SLOPE = 28
        self.ADDR_MX_CCW_COMPLIANCE_SLOPE = 29
        self.ADDR_MX_GOAL_POSITION = 30
        self.ADDR_MX_MOVING_SPEED = 32
        self.ADDR_MX_PRESENT_POSITION = 36
        self.ADDR_MX_PUNCH = 48
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.DXL_MOVING_STATUS_THRESHOLD = 10  # Threshold for detecting movement completion
        self.DXL_IDS = [1, 2, 3, 4]  # Motor IDs

        # Initialize portHandler and packetHandler
        self.portHandler = dxl.PortHandler(device_name)
        self.packetHandler = dxl.PacketHandler(protocol_version)

        # Open port and set baudrate
        if self.portHandler.openPort() and self.portHandler.setBaudRate(baudrate):
            print(f"Successfully opened port {device_name} and set baudrate")
        else:
            print(f"Failed to open port {device_name} and set baudrate")
            sys.exit(1)

    def enable_torque(self, motor_id):
        # Enable torque for a motor
        result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if result != dxl.COMM_SUCCESS:
            print(f"Failed to enable torque for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")

    def disable_torque(self, motor_id):
        # Disable torque for a specific motor
        result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if result != dxl.COMM_SUCCESS:
            print(f"Failed to disable torque for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")

    def deg_to_rot(self, deg):
        # Convert degrees to Dynamixel position units
        return int(deg * 1023 / 300)  # Assuming a 300° range of motion

    def set_speed(self, motor_id, speed):
        """
        Set the speed for a specific motor.
        :param motor_id: Motor ID to set the speed for.
        :param speed: Speed value between 0 and 1 (fraction of maximum speed).
        """
        if not (0 < speed <= 1):
            print(f"Invalid speed value for motor {motor_id}. Speed must be between 0 and 1.")
            return
        
        speed_value = int(speed * 1023)  # Scale speed to Dynamixel units (0–1023)
        result, error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_id, self.ADDR_MX_MOVING_SPEED, speed_value)
        if result != dxl.COMM_SUCCESS:
            print(f"Failed to set speed for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
        elif error != 0:
            print(f"Packet error while setting speed for motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
        else:
            print(f"Speed for motor {motor_id} set to {speed_value} (scaled from {speed * 100}%)")

    def set_goal_position(self, motor_id, position):
        # Set the target position for a motor
        result, error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_id, self.ADDR_MX_GOAL_POSITION, position)
        if result != dxl.COMM_SUCCESS:
            print(f"Failed to set goal position for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")

    def get_present_position(self, motor_id):
        # Attempt to read the present position for the specified motor
        position, result, error = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id, self.ADDR_MX_PRESENT_POSITION)
        if result != dxl.COMM_SUCCESS:
            print(f"Failed to read position for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
            return None
        return position

    def move_to_positions(self, goal_positions, speeds=None):
        # Move each motor to the target position specified in goal_positions
        if speeds is None:
            speeds = [1.0] * len(self.DXL_IDS)  # Default to maximum speed

        for motor_id, goal_pos, speed in zip(self.DXL_IDS, goal_positions, speeds):
            self.set_speed(motor_id, speed)  # Set speed for each motor
            self.set_goal_position(motor_id, goal_pos)  # Set goal position

        # Wait until all motors reach their goal positions
        for motor_id, goal_pos in zip(self.DXL_IDS, goal_positions):
            while True:
                current_position = self.get_present_position(motor_id)
                if current_position is None:
                    break  # Exit if we failed to read the position
                if abs(goal_pos - current_position) < self.DXL_MOVING_STATUS_THRESHOLD:
                    break  # Movement complete for this motor

    def close(self):
        # Disable torque and close port before exiting
        for motor_id in self.DXL_IDS:
            self.disable_torque(motor_id)
        self.portHandler.closePort()


# Basic usage demo:
if __name__ == "__main__":
    arm = RobotArm(device_name='COM4', baudrate=1000000)

    try:
        # Enable torque on all motors
        for motor_id in arm.DXL_IDS:
            arm.enable_torque(motor_id)

        # Define goal positions (in Dynamixel units) and speeds (0-1)
        goal_positions = [0, 512, 800, 512]  # Target positions for each motor
        speeds = [0.05, 0.1, 0.1, 0.1]  # Speeds for each motor (50%, 70%, 30%)

        # Move motors to their goal positions at the specified speeds
        arm.move_to_positions(goal_positions, speeds)

    finally:
        # Ensure proper cleanup
        arm.close()
