import matplotlib.pyplot as plt
import numpy as np

def plot_trajectory(trajectory):
    """
    Plot the Cartesian trajectory for debugging.
    Args:
        trajectory: List of Cartesian points [x, y, z].
    """
    trajectory = np.array(trajectory)
    plt.figure()
    plt.plot(trajectory[:, 0], trajectory[:, 2], '-o', label="Trajectory (X-Z)")
    plt.xlabel("X (m)")
    plt.ylabel("Z (m)")
    plt.title("Trajectory Visualization")
    plt.grid(True)
    plt.legend()
    plt.show()

def plot_robot_arm(arm, joint_angles):
    """
    Visualize the robot arm in the X-Z plane based on joint angles.
    Args:
        arm: Instance of the RobotArm class with DH parameters defined.
        joint_angles: List of joint angles in radians [θ1, θ2, θ3, θ4].
    """
    t = np.eye(4)  # Start with the identity matrix
    positions = [t[:3, 3]]  # Collect the origin of each link

    # Forward kinematics for each joint
    for i, (a, alpha, d, theta) in enumerate(arm.dh_params):
        theta += joint_angles[i]
        t_i = np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        t = np.dot(t, t_i)  # Multiply with the previous transformation matrix
        positions.append(t[:3, 3])  # Add the new position (x, y, z)

    # Extract X, Z coordinates for 2D plotting
    positions = np.array(positions)
    x_coords = positions[:, 0]
    z_coords = positions[:, 2]

    # Plot the robot arm
    plt.figure()
    plt.plot(x_coords, z_coords, '-o', label="Links")  # Plot links
    plt.scatter(x_coords[-1], z_coords[-1], color="red", label="End-Effector")  # Highlight end-effector
    plt.xlabel('X (m)')
    plt.ylabel('Z (m)')
    plt.title('Robot Arm Visualization (X-Z Plane)')
    plt.legend()
    plt.grid(True)
    plt.show()

# To use this function:
"""
print(f"Visualizing robot for joint angles (degrees): {[np.degrees(angle) for angle in joint_angles]}")
# Plot the robot arm
plot_robot_arm(arm, joint_angles)
"""