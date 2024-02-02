README for ur5


TERMINAL 1
cd Optitrack_ROS2/ros1_ws/docker
bash start_optitrack.sh
if docker not work automatically, run this in docker env: roslaunch optitrack_publisher optitrack_with_transform_to_base.launch
if still not work, catkin_make in /Optitrack_ROS2/ros1_ws, then: source devel/setup.bash
 

TERMINAL 2

cd ridgeback_ur5/docker_polyscope
bash start_docker.sh 

TERMINAL 3

cd ridgeback_ur5/docker_interface_ros
bash start_docker.sh 
roslaunch ur_lasa ur5_bringup_twist_controller.launch simu:=ON 

TERMINAL 4
cd DS_Robetarme/docker_planner
bash start_docker.sh interactive
catkin_make
roslaunch test_boustrophedon_with_robot path_planning.launch





BEFORE running terminal 5 connect the polyscope as following:

click on :
RUN PROGRAM

on the top left clikc on file ->load program
choose urcap_ros.urp and open

then click play button

after that goes to terminal 5


TERMINAL 5

cd Robetarme_iiwa/docker_ur5
bash start_docker.sh interactive
catkin_make
roslaunch ur5_controller twist.launch 


