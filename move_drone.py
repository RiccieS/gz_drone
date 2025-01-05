import subprocess
import time

def set_pose(name, x, y, z, roll=0, pitch=0, yaw=0):
    """
    Set the pose of a model in Gazebo.
    """
    pose_command = [
        "gz", "service", 
        "-s", "/world/drone_world/set_pose",
        "--reqtype", "gz.msgs.Pose", 
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "300",
        "--req", f'name: "{name}", position: {{ x: {x}, y: {y}, z: {z} }}, orientation: {{ x: {roll}, y: {pitch}, z: {yaw}, w: 1.0 }}'
    ]

    result = subprocess.run(pose_command, capture_output=True, text=True)
    if result.returncode == 0 and "data: true" in result.stdout:
        print(f"Drone moved to position ({x}, {y}, {z}) with orientation ({roll}, {pitch}, {yaw}).")
    else:
        print(f"Failed to move drone to position ({x}, {y}, {z}).")
        print("Error:", result.stderr, result.stdout)

# Movement functions
def take_off(name, height):
    set_pose(name, 0, 0, height)

def land(name):
    set_pose(name, 0, 0, 0)

def move_forward(name, current_x, current_y, current_z, distance):
    set_pose(name, current_x + distance, current_y, current_z)

def move_backward(name, current_x, current_y, current_z, distance):
    set_pose(name, current_x - distance, current_y, current_z)

def move_left(name, current_x, current_y, current_z, distance):
    set_pose(name, current_x, current_y - distance, current_z)

def move_right(name, current_x, current_y, current_z, distance):
    set_pose(name, current_x, current_y + distance, current_z)

def rotate(name, current_x, current_y, current_z, yaw_angle):
    set_pose(name, current_x, current_y, current_z, yaw=yaw_angle)

def move_up(name, current_x, current_y, current_z, distance):
    set_pose(name, current_x, current_y, current_z + distance)

def move_down(name, current_x, current_y, current_z, distance):
    set_pose(name, current_x, current_y, current_z - distance)

# Example of how to use these functions
def main():
    drone_name = "drone_model"
    current_x, current_y, current_z = 0, 0, 0

    print("Taking off...")
    take_off(drone_name, 5)
    current_z = 5
    time.sleep(2)

    print("Moving forward...")
    move_forward(drone_name, current_x, current_y, current_z, 5)
    current_x += 5
    time.sleep(2)

    print("Moving right...")
    move_right(drone_name, current_x, current_y, current_z, 5)
    current_y += 5
    time.sleep(2)

    print("Rotating...")
    rotate(drone_name, current_x, current_y, current_z, 90)  # Rotate by 90 degrees
    time.sleep(2)

    print("Landing...")
    land(drone_name)
    current_z = 0
    time.sleep(2)

    print("Showcase complete!")

if __name__ == "__main__":
    main()
