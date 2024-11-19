import sys
import time
import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler

class RobotArm:
    def __init__(self, device_name='COM5', baudrate=1000000, protocol_version=1.0, motor_ids=[1, 2, 3, 4]):
        # Address definitions
        self.ADDR_MX_TORQUE_ENABLE = 24
        self.ADDR_MX_GOAL_POSITION = 30
        self.ADDR_MX_PRESENT_POSITION = 36
        self.ADDR_MX_MOVING_SPEED = 32  # Speed address
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.DXL_MOVING_STATUS_THRESHOLD = 10

        self.motor_ids = motor_ids  # Dynamixel IDs
        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(protocol_version)

        # Open port and set baudrate
        if not (self.portHandler.openPort() and self.portHandler.setBaudRate(baudrate)):
            print("Failed to open port or set baudrate")
            sys.exit(1)
        else:
            print(f"Connected to {device_name} at {baudrate} baud.")

    def enable_torque(self):
        for motor_id in self.motor_ids:
            result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, motor_id, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE
            )
            if result != 0:
                print(f"Error enabling torque for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")

    def disable_torque(self):
        for motor_id in self.motor_ids:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, motor_id, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE
            )

    def set_speed(self, speed):
        """
        Set speed for all motors (0-1023).
        """
        speed_value = int(speed * 1023)  # Scale speed (e.g., 0.5 => 50% of max speed)
        for motor_id in self.motor_ids:
            self.packetHandler.write2ByteTxRx(self.portHandler, motor_id, self.ADDR_MX_MOVING_SPEED, speed_value)

    def move_to_position(self, positions):
        for motor_id, pos in zip(self.motor_ids, positions):
            self.packetHandler.write2ByteTxRx(self.portHandler, motor_id, self.ADDR_MX_GOAL_POSITION, int(pos))

    def get_position(self):
        positions = []
        for motor_id in self.motor_ids:
            pos, _, _ = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id, self.ADDR_MX_PRESENT_POSITION)
            positions.append(pos)
        return positions
    
    def get_torque(self):
        """
        Read the present load (torque) for each motor.
        Returns:
            List of torque values for each motor.
        """
        torque_values = []
        ADDR_MX_PRESENT_LOAD = 40  # Address for present load

        for motor_id in self.motor_ids:
            load, result, error = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id, ADDR_MX_PRESENT_LOAD)
            if result != 0:
                print(f"Error reading torque for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
                torque_values.append(None)  # Append None if there was an error
            else:
                # Convert to signed load (-512 to +511)
                if load > 1023:
                    load -= 1024
                torque_values.append(load)

        return torque_values

    def close(self):
        self.disable_torque()
        self.portHandler.closePort()
        print("Disconnected from the robot.")

def interpolate_trajectory(start, end, steps=100):
    """
    Create a linear interpolation trajectory between start and end points.
    """
    return np.linspace(start, end, steps)
