# LAUNCH: 
=> need to run those commands in separate terminals:
SIM: gz sim drone_world.sdf
Movement script: ./build/drone_movement
*** after every change in script, have to run cmake .. and make

turn on/off rotors:
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[0, 0, 0, 0]'

==> in .build before running "cmake .." => rm -rf *

LIDAR:
 gz topic -e -t /quadcopter/lidar
 gz topic -e -t /world/drone_world/dynamic_pose/info
 gz topic -l <== list all topics

*** after every change in .cpp script need to run "cd build" => "make"

GAZEBO's repository:
https://github.com/gazebosim/gz-sim/tree/gz-sim9/examples/worlds

GAZEBO's tutorials:
https://gazebosim.org/docs/harmonic/sensors/
https://github.com/gazebosim/docs/blob/master/harmonic/tutorials/sensors/sensor_tutorial.sdf



Ispiration:
pathfinding: https://github.com/yuchnw/quadSimulator?tab=readme-ov-file
hovering: https://www.youtube.com/watch?v=GK1t8YIvGM8&t=552s
control with controller: https://github.com/wilselby/ROS_quadrotor_simulator


list of libraries: \\wsl.localhost\Ubuntu-24.04\usr\share\gz


new notes to run:
CMD: gz service -l
/gazebo/resource_paths/add
/gazebo/resource_paths/get
/gazebo/resource_paths/resolve
/gazebo/worlds
/gui/camera/view_control
/gui/camera/view_control/reference_visual
/gui/camera/view_control/sensitivity
/gui/copy
/gui/follow
/gui/follow/offset
/gui/move_to
/gui/move_to/pose
/gui/paste
/gui/screenshot
/gui/view/collisions
/gui/view/com
/gui/view/frames
/gui/view/inertia
/gui/view/joints
/gui/view/transparent
/gui/view/wireframes
/marker
/marker/list
/marker_array
/sensors/marker
/sensors/marker/list
/sensors/marker_array
/server_control
/world/drone_model/create
/world/drone_model/create_multiple
/world/drone_model/disable_collision
/world/drone_model/enable_collision
/world/drone_model/light_config
/world/drone_model/remove
/world/drone_model/set_physics
/world/drone_model/set_pose
/world/drone_model/set_pose_vector
/world/drone_model/set_spherical_coordinates
/world/drone_model/visual_config
/world/drone_model/wheel_slip
/world/drone_world/control
/world/drone_world/control/state
/world/drone_world/create
/world/drone_world/create_multiple
/world/drone_world/declare_parameter
/world/drone_world/disable_collision
/world/drone_world/enable_collision
/world/drone_world/entity/system/add
/world/drone_world/generate_world_sdf
/world/drone_world/get_parameter
/world/drone_world/gui/info
/world/drone_world/level/set_performer
/world/drone_world/light_config
/world/drone_world/list_parameters
/world/drone_world/model/drone_model/link/body/sensor/imu_sensor/imu/set_rate
/world/drone_world/model/drone_model/link/body/sensor/lidar_sensor/scan/set_rate
/world/drone_world/playback/control
/world/drone_world/remove
/world/drone_world/scene/graph
/world/drone_world/scene/info
/world/drone_world/set_parameter
/world/drone_world/set_physics
/world/drone_world/set_pose
/world/drone_world/set_pose_vector
/world/drone_world/set_spherical_coordinates
/world/drone_world/state
/world/drone_world/state_async
/world/drone_world/system/info
/world/drone_world/visual_config
/world/drone_world/wheel_slip
CMD: gz service -s /world/drone_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 300 --req 'name: "drone_model", position: { x: 2, y: 2, z: 2 }, orientation: { w: 1.0 }'
data: true

CMD: gz service -s /world/drone_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 300 --req 'name: "drone_model", position: { x: 2, y: 2, z: 4 }, orientation: { w: 1.0 }'
data: true

python script: python3 move_drone.py