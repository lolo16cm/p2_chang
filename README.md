# p2_chang

## Name
- Carolyn Chang

## How to Run

### Part 1 — Autonomous Navigation
video: 
https://drive.google.com/file/d/1Z5hM9TWt8FfIUSvoOSN9uFCpVOT_Hj-t/view?usp=sharing
```
terminal 1:
roslaunch turtlebot_bringup minimal.launch --screen

terminal 2:
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch p2_chang p2a.launch
```

### Part 2 — Ball Follower (primary RGB)
video:
https://drive.google.com/file/d/1cxSsY_iUO0lGJYEFl-wQ5E5Io7sDcCHz/view?usp=sharing
```
terminal 1:
roslaunch turtlebot_bringup minimal.launch --screen

terminal 2:
roslaunch astra_camera astra.launch

terminal 3(output view):
rosrun image_view image_view image:=/image_converter/output_video

terminal 4:
roslaunch p2_chang p2b.launch
```


## L1 L2 L3 Locations
| Location | Coordinates (x, y) |
L1 = (0.106, -0.044)
<img width="300" height="400" alt="20260421_115408" src="https://github.com/user-attachments/assets/4e21e329-2fc1-473a-a2d1-e85b640dc227" />


L2 = (4.194, -0.214)
<img width="300" height="400" alt="20260422_093810" src="https://github.com/user-attachments/assets/60df2ce1-2854-4a65-83b7-c0b5924e46af" />


L3 = (3.011, -3.746)
<img width="300" height="400" alt="20260422_094005" src="https://github.com/user-attachments/assets/5c17d2e8-fb52-4191-bafd-6d3db0a32eea" />

