# <!-- Copyright (c) 2017, United States Government, as represented by the     -->
# <!-- Administrator of the National Aeronautics and Space Administration.     -->
# <!--                                                                         -->
# <!-- All rights reserved.                                                    -->
# <!--                                                                         -->
# <!-- The Astrobee platform is licensed under the Apache License, Version 2.0 -->
# <!-- (the "License"); you may not use this file except in compliance with    -->
# <!-- the License. You may obtain a copy of the License at                    -->
# <!--                                                                         -->
# <!--     http://www.apache.org/licenses/LICENSE-2.0                          -->
# <!--                                                                         -->
# <!-- Unless required by applicable law or agreed to in writing, software     -->
# <!-- distributed under the License is distributed on an "AS IS" BASIS,       -->
# <!-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         -->
# <!-- implied. See the License for the specific language governing            -->
# <!-- permissions and limitations under the License.                          -->


from utilities.utilities import *


def generate_launch_description():
    [discower_lab_urdf,  discower_lab_robot_description]  = get_urdf('urdf/model.urdf', 'discower_cosmo')

    return LaunchDescription([

        DeclareLaunchArgument("world"),

        # DISCOWER COSMO robot description
        Node(
            package="robot_state_publisher",
            namespace="discower_cosmo",
            executable="robot_state_publisher",
            name="lab_state_publisher",
            parameters=[{'robot_description': ParameterValue(discower_lab_robot_description) }],
            arguments=[discower_lab_urdf, "-topic /discower_cosmo"],
            condition=LaunchConfigurationEquals("world", "discower")
        ),
        # We must publish global transforms in case no robot has been spawned
        Node(
        package='framestore',
        namespace='',
        executable='global_transforms',
        name='global_transforms',
        )
    ])
