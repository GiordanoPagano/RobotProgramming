[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/j5ACTdgH)
# Second midterm -- Controller -- Deadline 19/12/2024
> [!CAUTION]
> It is **strictly prohibited** to alter any of the following files:
> - src/rp_eval/src/test_controller.cpp
> - src/rp_eval/src/test_controller_ca.cpp
> - .github/workflows/classroom.yml
> 
> **The repository tracker is set to trigger any alteration to these files. Any _unexpected_ change will lead to the invalidation of the assigment.**

## Task
The assignment consists of implementing two tasks:
- a controller generating the command velocity for the unicycle platform, which makes the robot follow the path computed by the planner;
- an obstacle avoidance procedure that modifies/substitutes the original control law to avoid unexpected obstacles.

You can compile with:
```sh
colcon build
```
For running Rviz2:
```sh
ros2 run rviz2 rviz2 -d rviz_configs/eval_assignment_2.rviz
```
For testing the first task (controller):
```sh
ros2 run rp_eval test_controller --ros-args -p "base_link_ns":="robot_1" -p "laser_ns":="laser_1" -p "image":="assets/diag_small.png" -p resolution:="0.1"
```
For testing the second task (controller + obstacle avoidance):
```sh
ros2 run rp_eval test_controller_ca --ros-args -p "base_link_ns":="robot_1" -p "laser_ns":="laser_1" -p "image":="assets/diag_small.png" -p resolution:="0.1"
```
The two tests take care of running the simulator, the map server, the planner, and the controller. In Rviz2 you can visualize in real-time how your solution is performing.

In `src/rp_controller/src/node.cpp` you have to solve some TODO to complete the ROS2 interface of the controller.

In `src/rp_controller/include/rp_controller/differential_drive_controller.h`, you have to tune (in the constructor) the control parameters.

In `src/rp_controller/src/differential_drive_controller.cpp`, you have to fill in the update function implementing the control law (first task) and the obstacle avoidance procedure (second task). In this function, you will find some hints inspired to our solution, but it is really up to you how you choose to control the unicycle!

Notice that the second test employs the image `assets/diag_small_obstacles.png` in the simulator, while the map server publishes `assets/diag_small.png`. This means that in reality (in the simulation) there exist small obstacles measured by the laser scanner, while the planner finds a path on the obstacle-free map (following this path will make the robot collide with them).

Wrapping up, you have a path to follow and a laser scan; consider both of them, be creative and avoid the obstacles! 


>[!TIP]
The only files you must work on are `src/rp_controller/src/node.cpp`, `src/rp_controller/include/rp_controller/differential_drive_controller.h`, and `src/rp_controller/src/differential_drive_controller.cpp`.


> [!IMPORTANT]
> - **Do not modify any file except `src/rp_controller/src/node.cpp`, `src/rp_controller/include/rp_controller/differential_drive_controller.h`, and `src/rp_controller/src/differential_drive_controller.cpp`**
> - You may push as many times as you want until the deadline \[**19/12/2024**\]
> - Run the two tests on your machine. If you can read **[eval_node]: Goal reached!** in your terminal, then your homework is **COMPLETED**.
> - Once you push on GitHub, tests are **not** performed. We will take care of running the evaluation on GitHub for all your solutions, and after a bit, you will receive a confirmation.

