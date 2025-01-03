import subprocess
import time

def set_pose(name, x, y, z, w=1.0):
    """
    Set the pose of a model in Gazebo.
    """
    pose_command = [
        "gz", "service", 
        "-s", "/world/drone_world/set_pose",
        "--reqtype", "gz.msgs.Pose", 
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "300",
        "--req", f'name: "{name}", position: {{ x: {x}, y: {y}, z: {z} }}, orientation: {{ w: {w} }}'
    ]

    result = subprocess.run(pose_command, capture_output=True, text=True)
    if result.returncode == 0 and "data: true" in result.stdout:
        print(f"Drone moved to position ({x}, {y}, {z}).")
    else:
        print(f"Failed to move drone to position ({x}, {y}, {z}).")
        print("Error:", result.stderr, result.stdout)

def main():
    drone_name = "drone_model"

    print("Taking off...")
    set_pose(drone_name, 0, 0, 5)  # Fly up to 5 meters
    time.sleep(2)  # Wait for 2 seconds

    print("Flying forward...")
    set_pose(drone_name, 5, 0, 5)  # Move forward to (5, 0, 5)
    time.sleep(2)

    print("Flying sideways...")
    set_pose(drone_name, 5, 5, 5)  # Move sideways to (5, 5, 5)
    time.sleep(2)

    print("Landing...")
    set_pose(drone_name, 5, 5, 0)  # Land at (5, 5, 0)
    time.sleep(2)

    print("Showcase complete!")

if __name__ == "__main__":
    main()
