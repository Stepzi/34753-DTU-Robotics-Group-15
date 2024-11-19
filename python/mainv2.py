from dynamixelArmv2 import RobotArm, interpolate_trajectory
import time

def main():
    arm = RobotArm('COM5', motor_ids=[1, 2, 3, 4])
    arm.enable_torque()

    try:
        # Define initial and target positions
        initial_positions = [512, 512, 512, 512]
        target_positions = [600, 500, 700, 300]

        # Set initial speed
        arm.set_speed(0.2)

        # Move to initial positions
        print("Moving to initial positions...")
        trajectory = interpolate_trajectory(arm.get_position(), initial_positions, steps=50)
        for step_positions in trajectory:
            arm.move_to_position(step_positions)
            time.sleep(0.05)

        # Monitor torque during movement
        print("Monitoring torque...")
        trajectory = interpolate_trajectory(initial_positions, target_positions, steps=50)

        for step_positions in trajectory:
            arm.move_to_position(step_positions)
            torques = arm.get_torque()
            print("Current torques:", torques)

            # Example condition: Check if any motor's torque exceeds a threshold
            # Could be used as condition check for other things
            if any(abs(torque) > 200 for torque in torques if torque is not None):
                print("Obstacle detected! Stopping movement.")
                break

            time.sleep(0.05)

        print("Finished movement.")

    finally:
        arm.close()

if __name__ == "__main__":
    main()
