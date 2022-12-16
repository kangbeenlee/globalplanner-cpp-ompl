ompl_move_base
==============

Global path planner plugin based on OMPL for move_base package

About
-----

This package provides global path planner plugin based on [OMPL](https://ompl.kavrakilab.org/) for [move_base](http://wiki.ros.org/move_base) package.
The move_base package supports any global planner adhering to the nav_core::BaseGlobalPlanner interface specified in the nav_core package.
OMPL, the Open Motion Planning Library, consists of many state-of-the-art sampling-based motion planning algorithms.

Attention: This package can be used as global planner in [gazebo-navigation repository](https://github.com/kangbeenlee/gazebo-navigation).
Tip: [Writing A Global Path Planner As Plugin in ROS](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS)

Usage
-----

1. **Make your own workspace and build**

    To use this planner, check it out in your local catkin workspace:
    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/kangbeenlee/ompl_move_base.git
    $ cd ..
    $ catkin_make
    ```

2. **Place plugin as shared object**
    
    Now the planner should be placed as shared object in the devel/lib folder.
    List the plugin with the command:
    ```
    $ rospack plugins --attrib=plugin nav_core
    ```

3. **Use the plugin**
    
    After 2nd step, you can use this plugin together with the move_base node.
    Then, use the plugin in your move_base_parameters.yaml file:
    ```
    base_global_planner: ompl_global_planner/OmplGlobalPlanner
    ```