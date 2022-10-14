# ecse373_f22_team1_robot_control

For team1 in ECSE373

## Project init command
To start the project, go to the root path and type command:

`roslaunch stdr_driver_node stdr_driver_node.launch`

If you want to use the GUI of STDR simulator:

`roslaunch stdr_driver_node stdr_driver_node_stdr.launch`

## Conrtol multiple Robot

Use different namespace to control different robot.

The default namespace is `robot0`

In order to control multiple robot, like `robot1`, use launch conmamnd with para: `namespace`

`roslaunch stdr_driver_node stdr_driver_node.launch namespace:=robot1`

## RQT GUI
To launch rqt and use robot-steering to control robot, please use command:

`rosrun rqt_gui rqt_gui`


![Screenshot from 2022-10-14 16-13-26](https://user-images.githubusercontent.com/47854126/195934396-83b071e1-1d9c-4999-b79b-d49b6a2d442f.png)
