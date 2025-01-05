import subprocess
import time
import json

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

def read_lidar_data():
    """
    Reads LiDAR data from the Gazebo topic.
    Returns a list of distances.
    """
    lidar_command = ["gz", "topic", "-e", "/lidar"]
    result = subprocess.run(lidar_command, capture_output=True, text=True)
    if result.returncode != 0:
        print("Failed to read LiDAR data")
        print("Error:", result.stderr)
        return []

    try:
        lidar_data = json.loads(result.stdout)
        return lidar_data.get("ranges", [])
    except json.JSONDecodeError as e:
        print("Error decoding LiDAR data:", e)
        return []

def find_clear_path(lidar_data, threshold=2.0):
    """
    Analyze LiDAR data to find the direction with the largest clear path.
    Returns the direction (in degrees) and the distance.
    """
    max_clear_distance = 0
    best_angle = None

    for i, distance in enumerate(lidar_data):
        if distance > max_clear_distance and distance >= threshold:
            max_clear_distance = distance
            best_angle = i

    return best_angle, max_clear_distance

def explore_room(drone_name, height):
    """
    Main exploration algorithm.
    """
    current_x, current_y, current_z = 0, 0, height

    print("Taking off...")
    set_pose(drone_name, current_x, current_y, current_z)
    time.sleep(2)

    while True:
        print("Scanning environment...")
        lidar_data = read_lidar_data()
        if not lidar_data:
            print("No LiDAR data received, stopping...")
            break

        print("Analyzing LiDAR data...")
        best_angle, max_clear_distance = find_clear_path(lidar_data, threshold=1.0)

        if max_clear_distance == 0:
            print("No clear path detected, rotating...")
            set_pose(drone_name, current_x, current_y, current_z, yaw=90)  # Rotate 90 degrees
            time.sleep(2)
        else:
            print(f"Clear path detected at angle {best_angle}° with distance {max_clear_distance}. Moving forward...")
            movement_distance = min(max_clear_distance, 1.0)  # Move up to 1 meter at a time
            if best_angle == 0:  # Straight forward
                current_x += movement_distance
            elif best_angle == 90:  # Right
                current_y += movement_distance
            elif best_angle == 180:  # Backward
                current_x -= movement_distance
            elif best_angle == 270:  # Left
                current_y -= movement_distance

            set_pose(drone_name, current_x, current_y, current_z)
            time.sleep(2)

        if max_clear_distance > 2.0 and best_angle == 0:
            print("Detected an open path. Flying through the door...")
            break

    print("Exploration complete!")

if __name__ == "__main__":
    explore_room("drone_model", height=1)
