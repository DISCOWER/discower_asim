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

cmake_minimum_required(VERSION 3.5)
project(discower_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC" )

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

find_package(ff_common REQUIRED)
find_package(ff_util REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(astrobee_gazebo REQUIRED)

#############
##  Build  ##
#############
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${astrobee_gazebo_INCLUDE_DIRS}
)

add_library(gazebo_model_plugin_thrusters SHARED
  src/gazebo_model_plugin_thrusters/gazebo_model_plugin_thrusters.cc
)

# target_link_libraries(gazebo_model_plugin_thrusters)
ament_target_dependencies(gazebo_model_plugin_thrusters rclcpp gazebo_dev gazebo_ros ff_common ff_util ff_msgs diagnostic_msgs geometry_msgs tf2)
ament_export_libraries(gazebo_model_plugin_thrusters)


#############
## Install ##
#############

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

# Install plugins
install(TARGETS
  gazebo_model_plugin_thrusters
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)


# Also install some other resources
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


# Mark launch files for installation
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE)

ament_package()

