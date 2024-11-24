launch: gz sim quadcopter_world.sdf

turn on/off rotors:
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'

gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[0, 0, 0, 0]'



GAZEBO's repository:
https://github.com/gazebosim/gz-sim/tree/gz-sim9/examples/worlds