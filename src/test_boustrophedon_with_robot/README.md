# test_boustrophedon_with_robot

this package will run service and send a robot velocity command, should use with iiwa_toolkit
MUST run all command one by one as follow:  
1. ```roslaunch iiwa_toolkit passive_track_gazebo.launch```
2.  !!! some time the client node need ```roslaunch ds_motion_generator load_test_code.launch``` to trigger it. don't know why////
3. ```roslaunch boustrophedon_server boustrophedon_server.launch```
4. use this begin point
5. ```rosrun test_boustrophedon_with_robot test_boustrophedon_with_robot_node```

the code for begin point
```
rostopic pub -r 10 /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: 0.9
      y: 0.0
      z: 0.5
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

