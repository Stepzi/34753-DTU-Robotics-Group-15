from dynamixelArmv2 import RobotArm
#from arucoDetection import detect_aruco_marker, map_grid_positions, detect_grid
#from tictactoe import TicTacToe
import numpy as np
import time
import cv2
import cv2.aruco as aruco

def initialize_robot(arm, neutral_position, steps=50):
    """
    Smoothly move the robot arm to a neutral position using interpolation and pause briefly at the position.
    Args:
        arm: Instance of RobotArm.
        neutral_position: List of Dynamixel goal positions for each joint.
        steps: Number of intermediate steps for smooth movement.
    """
    print("Initializing robot to neutral position...")
    arm.set_speed(0.2)  # Low speed for smooth initialization

    # Get the current positions
    current_positions = arm.get_position()
    print(f"Current Positions: {current_positions}")
    print(f"Neutral Position: {neutral_position}")

    # Interpolate positions for smooth movement
    trajectory = np.linspace(current_positions, neutral_position, steps)

    # Move through the interpolated positions
    for i, step_position in enumerate(trajectory):
        arm.move_to_position([int(pos) for pos in step_position])
        time.sleep(0.02)  # Small delay for smoother motion

    print("Robot initialized.")
    time.sleep(2)  # Pause for 2 seconds at the neutral position


def move_to_points(arm, points):
    """
    Move the end-effector to a set of 3D points using inverse kinematics.
    Args:
        arm: Instance of RobotArm.
        points: List of [x, y, z] positions in meters.
    """
    for i, point in enumerate(points):
        print(f"\nMoving to Point {i+1}: {point}")
        try:
            # Calculate joint angles for the target point
            joint_angles = arm.inverse_kinematics(point, elbow="up")
            print(f"Target Joint Angles (degrees): {[np.degrees(angle) for angle in joint_angles]}")

            # Convert joint angles to Dynamixel goal positions
            goal_positions = [int((np.degrees(angle) / 0.29) + 512) for angle in joint_angles]

            # Move all joints simultaneously
            arm.move_to_position(goal_positions)

            # Pause to allow the movement to complete
            time.sleep(2)  # Adjust delay based on hardware behavior

        except ValueError as e:
            print(f"Error moving to point {point}: {e}")


def main():
    # Initialize the robot arm and game
    arm = RobotArm()
    #game = TicTacToe()

    #cap = cv2.VideoCapture(0) # open webcam
    neutral_position = [512, 512, 512, 512] # home pos

    try:
        #arm.enable_torque()
        initialize_robot(arm, neutral_position)

        # Define a set of points in 3D space
        points = [
            [0.1, 0.0, 0.05],
            [0.15, 0.0, 0.1],
            [0.1, 0.1, 0.05],
            [0.05, -0.15, 0.00]
        ]
        move_to_points(arm, points)

    finally:
        arm.close()

if __name__ == "__main__":
    main()
