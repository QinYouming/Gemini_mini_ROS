# SVARM Project

## Packages under src/

`mavros`-communication with pixhawk4

`mocap_optitrack`-read the optitrack datas

`offboard`-generate the path and pass to px4

`mavlink`-the communication protocol of pixhawk4

## Normal Steps

1. `cd MaRS_Offboard`
2. `catkin build`
3. `source devel/setup.bash`
4. `roslaunch mocap_optitrack mocap.launch`
5. open a new terminal and move into MaRS_Offboard directory
6. `source devel/setup.bash`
7. `roslaunch mavros px4.launch`
8. open a new terminal and move into MaRS_Offboard directory
9. `source devel/setup.bash`
10. `rosrun offboard offboard_node`

after these steps, arm the pixhawk and switch into offboard mode through RC, the UAV will autonomous fly.



