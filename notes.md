launch: gz sim quadcopter_world.sdf

turn on/off rotors:
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[0, 0, 0, 0]'

LIDAR:
 gz topic -e -t /quadcopter/lidar
 gz topic -l <== list all topics


GAZEBO's repository:
https://github.com/gazebosim/gz-sim/tree/gz-sim9/examples/worlds

GAZEBO's tutorials:
https://gazebosim.org/docs/harmonic/sensors/
https://github.com/gazebosim/docs/blob/master/harmonic/tutorials/sensors/sensor_tutorial.sdf
