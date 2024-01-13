# DISCOWER Astrobee Simulation Components
The DISCOWER Astrobee Simulation package contains the following components:
* KTH Space Robotics Lab Gazebo World
* DISCOWER Free Flyers Model, including mesh files
* DISCOWER Free Flyers Gazebo Plugins: thrusters, PX4 SITL
* DISCOWER Launch files

## Installation
Make sure that you have ROS Humble installed on your system. Support for ROS Foxy was not tested.

0. Install all dependencies
```bash
sudo apt-get install build-essential git
```

1. Clone [DISCOWER Astrobee Simulation](https://github.com/DISCOWER/astrobee) into a clean workspace
```bash
cd ~/.
mkdir -p discower_ws
cd discower_ws/
git clone git@github.com:DISCOWER/astrobee.git src
```

2. Update the submodules the workspace
```bash
cd src/
git submodule update --init --depth 1 description/media
git submodule update --init --depth 1 discower
```

3. Install all dependencies
```bash
cd scripts/setup
./add_ros_repository.sh
sudo apt-get update
cd debians
./build_install_debians.sh
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

## Usage
To lanch the simulation, run the following command:
```bash
ros2 launch discower sim.launch.py
```

This simulation should spawn one free-flyer (Orion) into the workspace