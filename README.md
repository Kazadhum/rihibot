### Installation

This system was created for simulation using Webots R2023a and ROS Noetic. Make sure you have those two installed before continuing.

First, create the catkin workspace:

```
mkdir -p rihibot_ws/src
cd rihibot_ws
catkin_make
```

Install the ROS-Industrial `universal_robot` package from [here](https://github.com/ros-industrial/universal_robot). I like to make a directory for my miscellaneous dependencies packages.

```
cd rihibot_ws/src
mkdir misc_pkgs && cd misc_pkgs
git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

cd ../..

rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

catkin_make
```

We also need the `ur_e_webots` package, which can be copied from the local installation of Webots R2023a. Make sure the environment variable `WEBOTS_HOME` is set.

```
cd src/misc_pkgs
cp $WEBOTS_HOME/projects/robots/universal_robots/resources/ros_package/ur_e_webots .
cd ../..

rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

catkin_make
```

The [ATOM calibration framework](https://github.com/lardemua/atom) is also necessary:

```
cd rihibot_ws/src
git clone -b imu-calibration https://github.com/lardemua/atom

cd atom
sudo pip3 install -r requirements.txt
sudo apt-get install ros-noetic-ros-numpy qt5-default

cd ..
cd misc_pkgs
git clone https://github.com/miguelriemoliveira/rviz
cd ../..

catkin_make
```

Finally, clone this repository and compile:

```
cd rihibot_ws/src
mkdir -p robots && cd robots
git clone -b webots https://github.com/Kazadhum/rihibot.git

cd ../..
catkin_make
```
