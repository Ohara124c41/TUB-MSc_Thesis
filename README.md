# TUB-MSc_Thesis
Optimizing Agent Behavior and Mitigating User Cognitive Load for Mixed Human-Robot Teams.


### Speech with Gazebo Turtlebot (reference, as Gazebo crashes laptop)

each terminal needs to be sourced (or topics and launch files will not show)

Terminal 1:
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
Terminal 2:
```
rostopic echo /kws_data
```
Terminal 3:
```
roslaunch pocketsphinx kws.launch dict:=voice_cmd.dic kws:=voice_cmd.kwlist
```
Terminal 4:
```
rosrun pocketsphinx voice_control_example.py
````

### Speech with Turtlebot and confirm (WIP)

Again, in separate, sourced terminals:
```
roscore

roslaunch pocketsphinx speech_cmd.launch

roslaunch speech_cmd cmd.launch

rosrun turtlesim turtlesim_node
```

### Pocketsphinx Test
```
cd speech_recognition/

pocketsphinx_continuous -inmic yes -lm 8283.lm -dict 8283.dic
```
