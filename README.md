# mesh_visual_server

## Purpose of the Package
Insert object visual file into scene in RVIZ

## Roadmap
1. Create obj file from stl -by Blender - **done**
2. Implement obj file inside moveit PlanningSceneInterface - **done**
3. Create class of obj file - **done**
4. Create ROS2 client/service - **done**
5. Create ROS2 msg interface between client service - **done**
6. Create obj creation inside any namespace - **done**
7. Insert Paramter of creation obj - **done**
8. Create Operator ADD and REMOVE to obj - **done**
9. Create Operator Move obj - **done**
10. Create Operator Attach/Deatch to object- **not started**
11. Implement Continouse Collision checking - **not started**

## UML
![image](https://user-images.githubusercontent.com/122228012/219320037-0695e19e-f8c5-4b42-8ae1-a758dc923be0.png)

## Dependencies
1. install libgil
```https://libigl.github.io/```

2. moveit libraries
```https://moveit.picknik.ai/galactic/doc/tutorials/getting_started/getting_started.html```

## How to use
### Install
1. Install the msg package
```bash
colcon build --packages-select visual_srv_msg
```
2. Install the service and client
```bash
colcon build --packages-select visual_obj_server
```
### Run Example

1. Use moveit Panda demo enviorment in 1st terminal, Rviz should be opened and run
```bash
ros2 launch moveit2_tutorials move_group.launch.py
```
**now** source ws

2. Run service in 2nd terminal
```bash
ros2 run visual_obj_server server
```
3. Run client from 3rd terminal
```bash
ros2 run  visual_obj_server client add /home/dor/meshes/tray.obj tray1 / 1.0 1.1 1.3 1.
```
```
ros2 run  visual_obj_server client move  /home/dor/meshes/tray.obj tray1 / 0.0 0.4  0.4 0.0 0 0  1 world
```
```
ros2 run  visual_obj_server client add  /home/dor/meshes/tray.obj tray1 / 0.0 0.4  0.4 0.0 0 0  1 world
```
```
ros2 run  visual_obj_server client remove  /home/dor/meshes/tray.obj tray1 / 0.0 0.4  0.4 0.0 0 0  1 world
```


