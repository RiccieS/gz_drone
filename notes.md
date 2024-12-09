LAUNCH: 
=> need to run those commands in separate terminals:
SIM: gz sim quadcopter_world.sdf
LIDAR script: ./build/lidar_control
*** after every change in script, have to run cmake .. and make

turn on/off rotors:
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[0, 0, 0, 0]'

LIDAR:
 gz topic -e -t /quadcopter/lidar
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