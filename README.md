# Modular Lidar ROS Arduino

I have used:
* YDLIDAR X4 Lidar.
* Raspberry Pi 3 Model B. (or Raspberry Pi 4)

## 1. Install Ubuntu Server 20.04 LTS 64-bit (Ubuntu Noetic)

![](media\img\Install_Ubuntu_Server_20_04_LTS.png)

* SSH is enabled by default

#### User: ubuntu
#### Pass: ubuntu

When connect over SSH, the image force you to change the password of the user. I used to:
#### Pass: ubuntu20

## 2. Installing ROS

ROS (Robot Operation System) is a framework that facilitates the use of a wide variety of "packages" to control a robot. Those packages range all the way from motion control, to path planning, mapping, localization, SLAM, perception, and more. ROS provides a relatively simple interface with those packages, and the ability to of course create custom packages.

Note: The Raspberry Pi 4 is more computationally capable than its predecessors. However, installing ROS on the Pi3 is currently (as of December 2019) easier, and allegedly more reliable.

Following the steps in [ROS Website - Wiki - Ubuntu Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) or pdf inside folder: **media\pdf\noetic_Installation_Ubuntu - ROS Wiki.pdf**

## 3. Testing the lidar

This step was a bit of a doozy. It took me a while to figure out how to get the lidar to run. But I did! So hopefully you won't have to suffer too.

I am using the YDLIDAR X4 for this build. The first step is to install the necessary drivers. The driver is a ROS package.
* `cd catkin_ws/src`
* `git clone https://github.com/YDLIDAR/ydlidar_ros.git`
* `cd ..`
* `catkin_make`
* `source devel/setup.bash`
* Follow the directions from the repository, written below:
* * `roscd ydlidar_ros/startup`
* * `sudo chmod 777 ./*`
* * `sudo sh initenv.sh`

* Note: every reboot, you need to `source devel/setup.bash` from `cd catkin_ws` or consider to include in **.bashrc** on home user folder.

Test the lidar with `roslaunch ydlidar_ros X4.launch`. Visualize the scans in Rviz, by adding the topic `/scan`. 

It may look something like this! Background may vary :)

![Lidar Sample](media/lidar.jpg)

#### Add to .bashrc:
`source /home/ubuntu/catkin_ws/devel/setup.bash`

## 4. ROS + Arduino; Getting them to talk to each other.
As we know, the Raspberry Pi is the "brain" of our robot, perceiving the environment and planning in it. The Arduino, is simply used to control the motors of the robot. It doesn't do much thinking. So our goal here, is to get commands from the Raspberry Pi to the Arduino, so it'll be able to tell the motors how to move, accordingly. In high level, what we do is install *rosserial*, a ROS module that enables Arduino communication, on both the Raspberry Pi and the Arduino.
* Following the steps from [the ROS website](http://wiki.ros.org/rosserial_arduino/Tutorials), we start with installing the package. `sudo apt-get install ros-noetic-rosserial-arduino`, and then, `sudo apt-get install ros-noetic-rosserial`. If you are using a ROS version different from noetic, change the word `noetic` to your version.
* In the following commands, substitute `catkin_ws` with the name of your catkin workspace.
  ```
  cd catkin_ws/src
  git clone https://github.com/ros-drivers/rosserial.git
  cd catkin_ws
  catkin_make
  catkin_make install
  sudo adduser $USER dialout
  sudo adduser $USER tty
  ```
* In your Arduino IDE, install the rosserial library. I found it the easiest to just do it from the IDE itself. Search for `rosserial` in the Library Manager and install it.

And that's it!

For a test run, try the [HelloWorld example](http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World), from the examples included with the rosserial library. Flash the Arduino with it, and connect to the Raspberry Pi. To run it:

* On the Raspberry Pi `roscore`
* In a second Raspberry Pi terminal, `rosrun rosserial_python serial_node.py /dev/ttyS0`. Change `ttyS0` with the port of your Arduino. You can check the port by navigating to `~/dev/`, and observing which files disappear and re-appear when the Arduino is disconnected and connected. `rosrun rosserial_python serial_node.py _port:=/dev/ttyS0 _baud:=57600`
* In a third terminal, `rostopic echo chatter` to see the messages being sent.

### Serial problems with Raspberry Pi 3

- remove `console=serial0,115200` from `/boot/firmware/cmdline.txt` on Ubuntu
- disable the serial console: `sudo systemctl stop serial-getty@ttyS0.service && sudo systemctl disable serial-getty@ttyS0.service`

## 5. Installing Hector-SLAM
This part is exciting! We will now add the mapping and localization functionality to our robot. We use the Hector-SLAM package, since it enables us to create maps and localize ourselves with a Lidar alone. I found [this video by Tiziano Fiorenzani](https://www.youtube.com/watch?v=Qrtz0a7HaQ4) and the [official resources on the ROS website](http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot) helpful for setting Hector-SLAM up.
* Previously, we need to install a library Eigen 3, Qt 5, OpenCV 4 and . `sudo apt install libeigen3-dev && sudo apt install qt5-default && sudo apt-get install ros-noetic-cv-bridge && sudo apt-get install ros-noetic-vision-opencv`
* image transport, laser geometry, : `sudo apt-get install ros-noetic-theora-image-transport && sudo apt-get install  ros-noetic-laser-geometry`
* Clone the GitHub repository to your catkin workspace. Navigate to the `src` folder and run `git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git`.
* [This may fail!, see sub-bullet for work-arounds] Build ROS by running `catkin_make` and then sourcing `setup.bash` with `source ~/catkin_ws/devel/setup.bash`.
* * If your build gets stalled, or seems to be very slow. Do two things.
* * * Increase swap space to 1GB. [Follow this tutorial for instructions.](https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-16-04)
* * Run the build with `catkin_make -j2`

We need to make a couple of modifications to the Hector SLAM tutorial files in order for them to work with our setup. We first take note of the transformations available to us on the `\tf` topic, and the reference frames they use.
* Spin the lidar node, with `roslaunch ydlidar_ros lidar.launch`.
* Check the communication on the `/tf` topic with `rostopic echo /tf`
* I get only one transformation:
```
---                                                                          
transforms:                                                                         
  -                                                                          
    header:                                                                  
      seq: 0                                                                 
      stamp:                                                                 
        secs: 1578619851                                                     
        nsecs: 284012533                                                     
      frame_id: "/base_footprint"                                            
    child_frame_id: "/laser_frame"
    transform:                                             
      translation:                                         
        x: 0.2245                                          
        y: 0.0                                             
        z: 0.2                                             
      rotation:                                            
        x: 0.0                                             
        y: 0.0                                             
        z: 0.0                                             
        w: 1.0                                             
---                        
```
So we see that we have only two frames. Namely `/base_footprint` and `laser_frame`. We will update the file `~/catkin_ws/src/hector_slam/hector_mapping/launch/mapping_default.launch` to accommodate those.

* At the somewhat top of the file, change the first line to the second.

* * `  <arg name="odom_frame" default="nav"/>`

* * `  <arg name="odom_frame" default="base_footprint"/>`.

* At almost the very bottom of the file, change from/to:
* *   `<node pkg="tf" type="static_transform_publisher" name="map_nav_broadcaster" args="0 0 0 0 0 0 map nav 100"/>`
* *   `<node pkg="tf" type="static_transform_publisher" name="map_nav_broadcaster" args="0 0 0 0 0 0 base_footprint laser_frame 100"/>`

* Navigate to `~/catkin_ws/src/hector_slam/hector_slam_launch/launch/tutorial.launch`, and change from/to
* *   `<param name="/use_sim_time" value="true"/>`
* * `  <param name="/use_sim_time" value="false"/>`

This should do the trick! Try it out!

* In a first terminal run the lidar with `roslauch ydlidar_ros lidar.launch`
* In a second terminal run Hector SLAM with `roslaunch hector_slam_launch tutorial.launch`

You should be able to see the results in Rviz. Choose the `/map` topic to visualize the map that was created.

## 8. Lower Level Receive Position (That's where the Arduino comes in!)
We now want to create a ROS package that would allow ROS communication to receive position of the robot. 

Open four terminals, and run for each terminal one command in order and waiting the node init:
* `roscore`
* `roslauch ydlidar_ros X4.launch`
* `roslaunch hector_slam_launch tutorial.launch`
* `rosrun rosserial_python serial_node.py /dev/ttyS0`

And try the code in arduino DUE and check that the arduino is receiving the position.

```cpp
#include <ros.h>
ros::NodeHandle  nh;

#include <geometry_msgs/PoseStamped.h>
uint32_t msgPoseLastId;
geometry_msgs::PoseStamped msgPose;

void msgPoseStampCb(const geometry_msgs::PoseStamped & msgPoseStamp){
  msgPose = msgPoseStamp;
}

ros::Subscriber <geometry_msgs::PoseStamped> poseSub("slam_out_pose", &msgPoseStampCb);

void setup()
{
  SerialUSB.begin(115200);
  nh.initNode();
  nh.subscribe(poseSub);
}

void loop()
{
  nh.spinOnce();
  if(msgPose.header.seq != msgPoseLastId) {
    SerialUSB.print(msgPose.pose.position.x);
    SerialUSB.print(",");
    SerialUSB.print(msgPose.pose.position.y);
    SerialUSB.println();
    msgPoseLastId = msgPose.header.seq;
  }
  delay(10);
}
```

## 9. Launch files at startup!

At times you create a script and then you want to have the scripts controlled by systemd or in some cases you wish to have the scripts getting restarted by itself when it is killed due to some reason. In such cases systemd in Linux helps to configure services which can be managed. To do so follow the following steps.

### Systemd setup

For every command that you want to startup at boot, you can create a service and a script.

### /etc/systemd/system/roscore.service

```
cd /etc/systemd/system
sudo nano roscore.service
```

```
[Unit]
After=NetworkManager.service time-sync.target
[Service]
Type=simple
User=ubuntu
ExecStart=/usr/sbin/roscore-launcher
#Restart=on-failure
[Install]
WantedBy=multi-user.target
```

### /usr/sbin/roscore-launcher


```
cd /usr/sbin/
sudo nano roscore-launcher
```

```
#!/bin/bash
source /opt/ros/noetic/setup.sh
roscore
```

```
sudo chmod +x roscore-launcher
```

### Next make you script executable and enable your systemd scripts by exceuting the following in a terminal

```
sudo systemctl enable roscore.service
sudo chmod +x /usr/sbin/roscore-launcher
```

### So, to launch the lidar and the other rospackage you can launch, for example:

### LIDAR

### /etc/systemd/system/roslaunch-lidar.service

```
cd /etc/systemd/system
sudo nano roslaunch-lidar.service
```

```
[Unit]
After=NetworkManager.service time-sync.target roscore.service
[Service]
Type=simple
User=ubuntu
ExecStartPre=/bin/sleep 20
ExecStart=/usr/sbin/roslaunch-lidar
ExecStop=/bin/kill -s SIGTERM -$MAINPID &
[Install]
WantedBy=multi-user.target
```

### /usr/sbin/roslaunch-lidar


```
cd /usr/sbin/
sudo nano roslaunch-lidar
```

```
#!/bin/bash
source /home/ubuntu/catkin_ws/devel/setup.bash
export ROS_HOME=/home/ubuntu/.ros
roslaunch ydlidar_ros X4.launch
```

### Next make you script executable and enable your systemd scripts by exceuting the following in a terminal

```
sudo systemctl enable roslaunch-lidar.service
sudo chmod +x /usr/sbin/roslaunch-lidar
```

### HECTOR SLAM

### /etc/systemd/system/roslaunch-hector-slam.service

```
sudo nano /etc/systemd/system/roslaunch-hector-slam.service
```

```
[Unit]
After=NetworkManager.service time-sync.target roscore.service roslaunch-lidar.service
[Service]
Type=simple
User=ubuntu
ExecStartPre=/bin/sleep 20
ExecStart=/usr/sbin/roslaunch-hector-slam
ExecStop=/bin/kill -s SIGTERM -$MAINPID &
[Install]
WantedBy=multi-user.target
```

### /usr/sbin/roslaunch-hector-slam


```
sudo nano /usr/sbin/roslaunch-hector-slam
```

```
#!/bin/bash
source /home/ubuntu/catkin_ws/devel/setup.bash
export ROS_HOME=/home/ubuntu/.ros
roslaunch hector_slam_launch tutorial.launch
```

### Next make you script executable and enable your systemd scripts by exceuting the following in a terminal

```
sudo systemctl enable roslaunch-hector-slam.service
sudo chmod +x /usr/sbin/roslaunch-hector-slam
```

### ROS SERIAL

### /etc/systemd/system/roslaunch-serial-node.service

```
sudo nano /etc/systemd/system/roslaunch-serial-node.service
```

```
[Unit]
After=NetworkManager.service time-sync.target roscore.service roslaunch-lidar.service roslaunch-hector-slam.service
[Service]
Type=simple
User=ubuntu
ExecStartPre=/bin/sleep 20
ExecStart=/usr/sbin/roslaunch-serial-node
ExecStop=/bin/kill -s SIGTERM -$MAINPID &
[Install]
WantedBy=multi-user.target
```

### /usr/sbin/roslaunch-serial-node


```
sudo nano /usr/sbin/roslaunch-serial-node
```

```
#!/bin/bash
source /home/ubuntu/catkin_ws/devel/setup.bash
export ROS_HOME=/home/ubuntu/.ros
rosrun rosserial_python serial_node.py /dev/ttyS0
```

### Next make you script executable and enable your systemd scripts by exceuting the following in a terminal

```
sudo systemctl enable roslaunch-serial-node.service
sudo chmod +x /usr/sbin/roslaunch-serial-node
```

* PD: You can agrupate the all launcher files in a same launch file. I don't do that for flexibility and development, but you can if you want stable version, see  ROS documentation. 

## (Optionally) Remotely connecting to ROS
Something we would want to be able to do is to access the ROS communication messages from our laptop. There are a couple of steps to do here. On the robot check enviroment variables of ROS with `printenv | grep ROS`.

* On the Robot, find the `ROS_IP` and `ROSMASTER_URI`. These two things are the information both machines will need to communicate. Find the `ROS_IP` by running `ifconfig`. 

* On the Master (robot), run `roscore &`. 
* * `ROS_IP` is its own IP.
* * `ROS_MASTER_URI` is HTTP://<its own IP>:11311.

In this example (the IPs would probably be different in your network), on the robot, we set: `export ROS_IP=192.168.1.5 export ROS_MASTER_URI=http://192.168.1.5:11311`. Consider to include in **.bashrc** on home user folder.

#### Add to .bashrc:
`export ROS_IP=192.168.1.5`
`export ROS_MASTER_URI=http://192.168.1.5:11311`