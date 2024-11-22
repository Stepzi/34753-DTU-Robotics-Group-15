import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler

class RobotArm:
    def __init__(self, device_name='COM5', baudrate=1000000, protocol_version=1.0, motor_ids=[1, 2, 3, 4]):
        # Dynamixel control table addresses
        self.ADDR_MX_TORQUE_ENABLE = 24
        self.ADDR_MX_GOAL_POSITION = 30
        self.ADDR_MX_PRESENT_POSITION = 36
        self.ADDR_MX_MOVING_SPEED = 32
        self.ADDR_MX_PRESENT_LOAD = 40  # Address for present load (torque)
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # DH parameters: [a, alpha, d, theta] for each joint
        self.dh_params = [
            [0, np.pi / 2, 0.05, 0],  # Joint 1
            [0.093, 0, 0, np.pi / 2],  # Joint 2
            [0.093, 0, 0, 0],  # Joint 3
            [0.08, 0, 0, 0],  # Joint 4 (End-effector)     [0.05, 0, 0, 0]
        ]

        self.motor_ids = motor_ids
        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(protocol_version)

        # Open port and set baudrate
        if not (self.portHandler.openPort() and self.portHandler.setBaudRate(baudrate)):
            raise Exception("Failed to open port or set baudrate.")
        print(f"Connected to {device_name} at {baudrate} baud.")

    def enable_torque(self):
        for motor_id in self.motor_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)

    def disable_torque(self):
        for motor_id in self.motor_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE)

    def set_speed(self, speed):
        """
        Set the speed for all motors (value between 0 and 1).
        """
        speed_value = int(speed * 1023)
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
        torques = []
        for motor_id in self.motor_ids:
            load, result, error = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id, self.ADDR_MX_PRESENT_LOAD)
            if result != 0:
                print(f"Error reading torque for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
                torques.append(None)
            else:
                # Convert to signed load (-512 to +511)
                if load > 1023:
                    load -= 1024
                torques.append(load)
        return torques

    def forward_kinematics(self, joint_angles):
        t = np.eye(4)
        for i, (a, alpha, d, theta) in enumerate(self.dh_params):
            theta += joint_angles[i]
            t_i = np.array([
                [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ])
            t = np.dot(t, t_i)
        return t

    def inverse_kinematics(self, target_position, elbow="up"):
        x, y, z = target_position
        l1, l2, l3 = self.dh_params[0][2], self.dh_params[1][0], self.dh_params[2][0]

        theta1 = np.arctan2(y, x)
        d = np.sqrt(x**2 + y**2) - l1
        r = np.sqrt(d**2 + z**2)

        cos_theta3 = (r**2 - l2**2 - l3**2) / (2 * l2 * l3)
        if abs(cos_theta3) > 1.0:
            raise ValueError("Target out of reach")
        theta3 = np.arccos(cos_theta3)
        if elbow == "down":
            theta3 = -theta3

        theta2 = np.arctan2(z, d) - np.arctan2(l3 * np.sin(theta3), l2 + l3 * np.cos(theta3))
        theta4 = 0  # Fixed orientation for simplicity

        return [theta1, theta2, theta3, theta4]

    def move_to_xyz(self, target_position, elbow="up"):
        joint_angles = self.inverse_kinematics(target_position, elbow)
        positions = [int(np.degrees(angle) / 0.29) + 512 for angle in joint_angles]
        self.move_to_position(positions)

    def close(self):
        self.disable_torque()
        self.portHandler.closePort()
        print("Disconnected from the robot.")

def interpolate_trajectory(start, end, steps=100):
    """
    Create a linear interpolation trajectory between start and end points.
    """
    return np.linspace(start, end, steps)