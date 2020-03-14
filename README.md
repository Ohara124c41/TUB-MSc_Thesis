# TUB-MSc_Thesis
Optimizing Agent Behavior and Mitigating User Cognitive Load for Mixed Human-Robot Teams.

This README covers the basic setup for using PocketSphinx with ROS Kinetic (Gazebo/RViz) and Unity.
# SET-UP
The current version has been used to setup the virtual environment (ROS, etc.) on two machines (unless ROSBridge and Websocket Protocol are used with an internal connection).

A docker image will be created later to bypass the section below. For the time being, please follow these steps.


Assuming this project is cloned and utilizing the file architecture:
```
cd speech_recognition
git clone https://github.com/cmusphinx/sphinxbase

cd sphinxbase
./autogen.sh
./configure
make clean all
make check
sudo make install
export LD_LIBRARY_PATH=/usr/local/lib


git clone https://github.com/cmusphinx/pocketsphinx
cd pocketsphinx/
./autogen.sh
./configure
make clean all
make check
sudo make install

git clone https://github.com/cmusphinx/sphinxtrain
cd sphinxtrain/
./autogen.sh
./configure
make clean all
make check
sudo make install

svn checkout svn://svn.code.sf.net/p/cmusphinx/code/trunk cmusphinx-code
cd cmusphinx-code/
cd cmumlmtk/
./autogen.sh
./configure
make clean all
make check
sudo make install


sudo apt-get install ros-kinetic-ar-track-alvar-msgs
cd catkin_ws/
source devel/setup.bash
roslaunch pocketsphinx speech_cmd.launch 

roslaunch pocketsphinx kws.launch dict:=/home/ohara/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic kws:=/home/ohara/catkin_ws/src/pocketsphinx/demo/voice_cmd.kwlist input:=/home/ohara/catkin_ws/src/pocketsphinx/demo/goforward.raw
roslaunch turtlebot_gazebo turtlebot_world.launch


sudo dpkg -i '/home/ohara/Downloads/pocketsphinx-utils_0.8.0+real-0ubuntu6_amd64.deb' 
sudo dpkg -i '/home/ohara/Downloads/libpocketsphinx1_0.8-5_amd64.deb' 
sudo dpkg -i '/home/ohara/Downloads/libsphinxbase1_0.8-6_amd64.deb' 
sudo dpkg -i '/home/ohara/Downloads/libpocketsphinx1_0.8-5_amd64.deb' 
sudo dpkg -i '/home/ohara/Downloads/pocketsphinx-utils_0.8.0+real-0ubuntu6_amd64.deb' 
sudo apt-get install libsphinxbase1
sudo apt-get -f install
sudo apt-get install libpocketsphinx1
sudo apt-get install pocketsphinx
sudo dpkg -i '/home/ohara/Downloads/gstreamer0.10-pocketsphinx_0.8.0+real-0ubuntu6_amd64.deb' 
sudo dpkg -i '/home/ohara/Downloads/gstreamer1.0-plugins-base_1.4.4-2+deb8u2_amd64.deb' 
sudo pip install pyaudio
sudo pip install pocketsphinx
sudo apt-get install python-gst-1.0
sudo apt install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0 ffmpeg libav-tools
sudo apt install python-gst0.10
sudo apt-get install gstreamer0.10-gconf

sudo apt-get install pyttsx
pip install pyttsx
sudo apt-get install libpulse-dev
sudo apt-get install bison

mkdir pocketsphinx
cd pocketsphinx
wget http://sourceforge.net/projects/cmusphinx/files/sphinxbase/0.8/sphinxbase-0.8.tar.gz
wget http://www.repository.voxforge1.org/downloads/de/Trunk/Lexicon/Lexicon.tgz
wget http://www.repository.voxforge1.org/downloads/de/Trunk/AcousticModels/Sphinx_AcousticModel.tgz
wget http://sourceforge.net/projects/cmusphinx/files/pocketsphinx/0.8/pocketsphinx-0.8.tar.gz
tar xvf pocketsphinx-0.8.tar.gz
tar xvf sphinxbase-0.8.tar.gz
tar xvf Lexicon.tgz
tar xvf Sphinx_AcousticModel.tgz

cd sphinxbase-0.8/
make clean
make
catkin_make
sudo make install
sudo catkin_make install

cd pocketsphinx-0.8/
catkin_make clean
catkin_make
catkin_make install

cd catkin_ws/
catkin_make install

rosrun pocketsphinx voice_control_example.py



gst-launch -v -m audiotestsrc ! audioconvert ! audio/x-raw-int,channels=2,width=8,depth=8 ! level ! fakesink silent=TRUE

```



### Pocketsphinx Test
```
cd speech_recognition/

pocketsphinx_continuous -inmic yes -lm 8283.lm -dict 8283.dic
```

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

### Speech with Turtlebot and confirm 

```
roscore

roslaunch pocketsphinx speech_cmd.launch

roslaunch speech_cmd cmd.launch

rosrun turtlesim turtlesim_node
```


