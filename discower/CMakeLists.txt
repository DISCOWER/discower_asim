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
project(discower)

## Compile as C++14, supported in ROS Kinetic and newer

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#############
## Install ##
#############

# Install Python modules
# ament_python_install_package("utilities")

# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# resources will be installed separately
# install(DIRECTORY resources/ DESTINATION resources)

# Mark launch files for installation
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE)

# Mark resource files for installation
# install(DIRECTORY resources
#   DESTINATION share/${PROJECT_NAME}
#   PATTERN ".svn" EXCLUDE)

# install(PROGRAMS scripts/apk_print_version.sh
# 				  scripts/check_env.sh
#           scripts/print_version.sh
#           scripts/cpu_print_version.sh
#           scripts/launch_gds.sh
#   DESTINATION lib/${PROJECT_NAME})

# install(PROGRAMS scripts/env_wrapper.sh DESTINATION ${CMAKE_INSTALL_PREFIX} )

ament_package()