# GimbalRotor

## Launch

1. pid controller launching

```
roslaunch gimbalrotor bringup.launch airframe:=bi real_machine:=false sim:=true launch_gazebo:=true headless:=false

```

2. run tele_keyboard node  

```
rosrun aerial_robot_base keyboard_command.py
```

### common commands

- r: motor arming (please do this before takeoff)
- t: takeoff (this can be received by robot only after motor arming)
- l: landing
- f: force landing (without xy position control, robot will descend slowly)
- h: halt (stop motor immediately )
