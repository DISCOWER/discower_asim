# DISCOWER Astrobee Simulation Components
The DISCOWER Astrobee Simulation package contains the following components:
* KTH Space Robotics Lab Gazebo World
* DISCOWER Free Flyers Model, including mesh files
* DISCOWER Free Flyers Gazebo Plugins: thrusters, PX4 SITL
* DISCOWER Launch files

![DISCOWER Astrobee Simulation](https://github.com/DISCOWER/discower_asim/blob/main/discower/doc/images/simulator.png)

## Installation
Make sure that you have ROS Humble installed on your system. Support for ROS Foxy was not tested. Make sure that you have SSH Keys synced with GitHub. If you don't have SSH Keys, follow [this tutorial](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) to generate them.

0. Install all dependencies
```bash
sudo apt-get install build-essential git
```

1. Clone [DISCOWER Astrobee Simulation](https://github.com/DISCOWER/astrobee) into a clean workspace
```bash
cd ~/.
mkdir discower_ws
cd discower_ws/
git clone git@github.com:DISCOWER/astrobee.git src
```

2. Update the submodules the workspace
```bash
cd src/
git submodule update --init --depth 1 description/media
git submodule update --init discower
git submodule update --init --recursive submodules/px4
```

3. Install all dependencies
```bash
cd scripts/setup
./add_ros_repository.sh
sudo apt-get update
cd sources
./build_install_sources.sh
cd ../
./install_desktop_packages.sh
sudo rosdep init
rosdep update
```

4. Build the workspace with `colcon build --symlink-install` (Note: for systems with 16GB of RAM or less, use `colcon build --symlink-install --executor sequential`)
```bash
cd ~/discower_ws/
colcon build --symlink-install
```

5. Source the workspace
```bash
source install/local_setup.bash
```

6. Make sure that you have the following Gazebo paths in your system:
```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/discower_ws/src/submodules/px4/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
```
to ensure that the PX4 plugins are found by Gazebo.

## Usage
To launch the simulation, run the following command:
```bash
ros2 launch discower sim.launch.py
```

This simulation should spawn one free-flyer (Orion) into the workspace with a SITL window running on the same terminal.