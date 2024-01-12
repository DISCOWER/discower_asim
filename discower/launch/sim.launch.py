# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

from utilities.utilities import *


def generate_launch_description():
    return LaunchDescription([
        launch_arg("robot",  default_value=os.getenv("ASTROBEE_ROBOT", "sim"), description="Robot name"),
        launch_arg("world",  default_value="discower",                         description="World name"),
        launch_arg("output", default_value="log", description="Where nodes should log"),
        launch_arg("spurn",  default_value="",    description="Prevent a specific node"),
        launch_arg("nodes",  default_value="",    description="Launch specific nodes"),
        launch_arg("extra",  default_value="",    description="Inject additional node"),
        launch_arg("debug",  default_value="",    description="Debug node group"),

        # Physics engine options: ode (gazebo default), bullet, simbody, dart
        launch_arg("physics", default_value="ode",   description="Choose physics engine"),
        launch_arg("sim",     default_value="local", description="SIM IP address"),
        launch_arg("llp",     default_value="local", description="LLP IP address"),
        launch_arg("mlp",     default_value="local", description="MLP IP address"),
        launch_arg("rec",     default_value="",      description="Record local data "),
        launch_arg("dds",     default_value="false",  description="Enable DDS"),
        launch_arg("gtloc",   default_value="true", description="Use Ground Truth Localizer"),
        launch_arg("perch",   default_value="false", description="Start in the perch position"),

        # General options
        launch_arg("rviz",   default_value="false",  description="Start Rviz visualization"),
        launch_arg("sviz",   default_value="true",  description="Start Gazebo visualization"),
        launch_arg("speed",  default_value="1",      description="Speed multiplier"),
        launch_arg("sdebug", default_value="false",  description="Debug simulator "),

        launch_arg("orion", default_value="true", description="Insert Orion robot"),
        launch_arg("apollo", default_value="false", description="Insert Apollo robot"),
        launch_arg("leo", default_value="false", description="Insert Leo robot"),
        launch_arg("orion_pose",  default_value="0.5 0.0 0 0 0 0", description="Overwrite orion's pose"),
        launch_arg("apollo_pose",  default_value="1 1 0 0 0 0", description="Overwrite apollo's pose"),
        launch_arg("leo_pose",  default_value="0.5 1 0 0 0 0", description="Overwrite leo's pose"),
        # Make sure all environment variables are set for controller
        # Override the robot and world environment variables all the time. The
        # environment variables are the default if they are set. So in this
        # case we are overriding the environment variables with themselves.
        # Ros launch arguments override the environment variable which is what
        # this will do.
        SetEnvironmentVariable(name="ASTROBEE_ROBOT", value=os.getenv("ASTROBEE_ROBOT", LaunchConfiguration("robot"))),
        SetEnvironmentVariable(name="ASTROBEE_WORLD", value=os.getenv("ASTROBEE_WORLD", LaunchConfiguration("world"))),
        SetEnvironmentVariable(name="ASTROBEE_CONFIG_DIR",    value=os.getenv("ASTROBEE_CONFIG_DIR",    get_path("config"))),
        SetEnvironmentVariable(name="ASTROBEE_RESOURCE_DIR",  value=os.getenv("ASTROBEE_RESOURCE_DIR",  get_path("resources"))),
        SetEnvironmentVariable(name="ROSCONSOLE_CONFIG_FILE", value=os.getenv("ROSCONSOLE_CONFIG_FILE", get_path("resources/logging.config"))),
        # Update the environment variables relating to absolute paths
        SetEnvironmentVariable(name="ASTROBEE_CONFIG_DIR",
                               value=os.getenv("ASTROBEE_CONFIG_DIR", "/home/astrobee/native/config"),
                               condition=LaunchConfigurationNotEquals("sim", "local")),
        SetEnvironmentVariable(name="ASTROBEE_RESOURCE_DIR",
                               value=os.getenv("ASTROBEE_RESOURCE_DIR", "/home/astrobee/native/resources"),
                               condition=LaunchConfigurationNotEquals("sim", "local")),
        SetEnvironmentVariable(name="ROSCONSOLE_CONFIG_FILE",
                               value=os.getenv("ROSCONSOLE_CONFIG_FILE", "/home/astrobee/native/resources/logging.config"),
                               condition=LaunchConfigurationNotEquals("sim", "local")),

        SetEnvironmentVariable(name="DISPLAY", value=":0",
                               condition=LaunchConfigurationNotEquals("sim", "local")),
        SetEnvironmentVariable(name="ROS_IP", value=LaunchConfiguration("sim"),
                               condition=LaunchConfigurationNotEquals("sim", "local")),
        # Declare our global logging format
        SetEnvironmentVariable(name="RCUTILS_CONSOLE_OUTPUT_FORMAT",
            value="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"),
        # Always launch on the local machine
        #   <group>
        #     <machine name ="local" address="localhost" default="true"/>
        # Start the descriptions (ISS, dock, granite) for visualization purposes
        # Start ground controller services
        IncludeLaunchDescription(
            get_launch_file("launch/controller/rviz.launch.py", "discower"),
            condition=IfCondition(LaunchConfiguration("rviz")),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),

        # Start the simulator
        IncludeLaunchDescription(
            get_launch_file("launch/controller/sim_start.launch.py", "discower"),
            launch_arguments={
                "world"  : LaunchConfiguration("world"),
                "sviz"   : LaunchConfiguration("sviz"),
                "speed"  : LaunchConfiguration("speed"),
                "debug"  : LaunchConfiguration("debug"),
                "physics": LaunchConfiguration("physics"),
            }.items(),
        ),
        # Launch files
        IncludeLaunchDescription(
            get_launch_file("launch/controller/descriptions.launch.py", "discower"),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),
        # Insert Orion
        IncludeLaunchDescription(
            get_launch_file("launch/spawn.launch.py", "discower"),
            launch_arguments={
                "robot" : LaunchConfiguration("robot"),      # Type of robot
                "world" : LaunchConfiguration("world"),      # Execution context
                "ns"    : "orion",                           # Robot namespace
                "output": LaunchConfiguration("output"),     # Output for logging
                "pose"  : LaunchConfiguration("orion_pose"), # Initial robot pose
                "spurn" : LaunchConfiguration("spurn"),      # Prevent node
                "nodes" : LaunchConfiguration("nodes"),      # Launch node group
                "extra" : LaunchConfiguration("extra"),      # Inject extra nodes
                "debug" : LaunchConfiguration("debug"),      # Debug a node set
                "sim"   : LaunchConfiguration("sim"),        # SIM IP address
                "llp"   : LaunchConfiguration("llp"),        # LLP IP address
                "mlp"   : LaunchConfiguration("mlp"),        # MLP IP address
                "dds"   : LaunchConfiguration("dds"),        # Enable DDS
                "gtloc" : LaunchConfiguration("gtloc"),      # Use Ground Truth Localizer
            }.items(),
            condition=IfCondition(LaunchConfiguration("orion")),
        ),
        # Insert Apollo
        IncludeLaunchDescription(
            get_launch_file("launch/spawn.launch.py", "discower"),
            launch_arguments={
                "robot" : LaunchConfiguration("robot"),       # Type of robot
                "world" : LaunchConfiguration("world"),       # Execution context
                "ns"    : "apollo",                           # Robot namespace
                "output": LaunchConfiguration("output"),      # Output for logging
                "pose"  : LaunchConfiguration("apollo_pose"), # Initial robot pose
                "spurn" : LaunchConfiguration("spurn"),       # Prevent node
                "nodes" : LaunchConfiguration("nodes"),       # Launch node group
                "extra" : LaunchConfiguration("extra"),       # Inject extra nodes
                "debug" : LaunchConfiguration("debug"),       # Debug a node set
                "sim"   : LaunchConfiguration("sim"),         # SIM IP address
                "llp"   : LaunchConfiguration("llp"),         # LLP IP address
                "mlp"   : LaunchConfiguration("mlp"),         # MLP IP address
                "dds"   : LaunchConfiguration("dds"),         # Enable DDS
                "gtloc" : LaunchConfiguration("gtloc"),       # Use Ground Truth Localizer
            }.items(),
            condition=IfCondition(LaunchConfiguration("apollo")),
        ),
        # Insert Leo
        IncludeLaunchDescription(
            get_launch_file("launch/spawn.launch.py", "discower"),
            launch_arguments={
                "robot" : LaunchConfiguration("robot"),       # Type of robot
                "world" : LaunchConfiguration("world"),       # Execution context
                "ns"    : "leo",                              # Robot namespace
                "output": LaunchConfiguration("output"),      # Output for logging
                "pose"  : LaunchConfiguration("leo_pose"),    # Initial robot pose
                "spurn" : LaunchConfiguration("spurn"),       # Prevent node
                "nodes" : LaunchConfiguration("nodes"),       # Launch node group
                "extra" : LaunchConfiguration("extra"),       # Inject extra nodes
                "debug" : LaunchConfiguration("debug"),       # Debug a node set
                "sim"   : LaunchConfiguration("sim"),         # SIM IP address
                "llp"   : LaunchConfiguration("llp"),         # LLP IP address
                "mlp"   : LaunchConfiguration("mlp"),         # MLP IP address
                "dds"   : LaunchConfiguration("dds"),         # Enable DDS
                "gtloc" : LaunchConfiguration("gtloc"),       # Use Ground Truth Localizer
            }.items(),
            condition=IfCondition(LaunchConfiguration("leo")),
        ),
        ]
    )
