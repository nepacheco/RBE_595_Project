# RBE_595_Project

## Modules to import before running
#### Make sure all imports are for python 2 for ROS

`python -m pip install -U pip`

`python -m pip install -U matplotlib`

`pip -m install numpy`


`pip -m install numpy-stl`

## Add models directory to Gazebo path
add following line to ~/.bashrc but change it to be relative to your directories

`export GAZEBO_MODEL_PATH="{PATH_TO_PROJECT}/RBE_595_Project/pcl_node/models/"`

## Running the code
First run catkin make in your workspace
* Note: must be catkin make not catkin build, if you previously ran build, delete your build and devel folders and run catkin make

In pcl_node/launch/camera.launch, uncomment the line for the object you want to display. Then run the launch file

`roslaunch pcl_node camera.launch`

Next run the pcl_node with the parameter for the shape to segment as follows

    1: Cylinder
    2: Sphere
    3: Cone
    4: Box
    
`rosrun pcl_node ShapeFinder _shape:=SHAPE_NUMBER_HERE`

Lastly, run the python node to display the grasps

`roslaunch shape_node shape_node.launch`
     
