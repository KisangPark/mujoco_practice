# Mujoco Test

https://github.com/user-attachments/assets/1b57a839-f2ee-47e9-88d3-99dc89443523

## Setup
1. pip3 install mujoco
2. pip3 install glfw

## Mesh sources
- a0912 robot arm (doosan robot)
    - https://github.com/DoosanRobotics/doosan-robot/tree/main
- 2f85 parallel gripper
    - https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotiq_2f85
    - (Educational purposes)

## Tutorial
### General control </br>
- left click & drag to rotate</br>
- right drag to move</br>
- scroll to zoom in/out</br>
</br>


### viewer_camera_test.py</br>
- open xml file with mujoco_viewer</br>
    - Keyboard option control</br>
    - github repository: https://github.com/rohanpsingh/mujoco-python-viewer </br>


https://github.com/user-attachments/assets/ee732431-0dce-44c2-adb2-e711c3e98691



### keyboard_teleop.py</br>
- glfw window creation</br>
    - use python glfw library</br>
    - Multiple camera views: interactive (free) / ego-centric (fixed)</br>
- keyboard teleop with glfw callback</br>
    - 4-DOF (4 joints controllable) </br>
    - A,D: z-axis rotation</br>
    - N,M: bottom y-axis joint rotation</br>
    - K,L: middle y-axis joint rotation</br>
    - O,P: top y-axis joint rotation</br>
    - U,I: gripper</br>
    - R to reset

