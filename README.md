# DeepRacer Status LED Package

## Overview

The DeepRacer Status LED ROS package creates the *status_led_node* which is part of the core AWS DeepRacer application and will be launched from the deepracer_launcher. More details about the application and the components can be found [here](https://github.com/aws-racer/aws-deepracer-launcher).

This node contains the logic of the blink and solid light effects for status led lights by the side of the DeepRacer device. It provides services and functions to enable/disable the RGB GPIO ports for led lights, start and stop the particular effect on the led lights.

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages and libraries installed to run the status_led_pkg. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-racer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The status_led_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

* *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the status_led_pkg on the DeepRacer device:

        git clone https://github.com/aws-racer/aws-deepracer-status-led-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-status-led-pkg
        rosws update

1. Resolve the status_led_pkg dependencies:

        cd ~/deepracer_ws/aws-deepracer-status-led-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the status_led_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-status-led-pkg && colcon build --packages-select status_led_pkg deepracer_interfaces_pkg

## Usage

Although the *status_led_node* is built to work with the AWS DeepRacer application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built status_led_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-status-led-pkg/install/setup.bash 

1. Launch the status_led_node using the launch script:

        ros2 launch status_led_pkg status_led_pkg_launch.py

## Launch Files

The  status_led_pkg_launch.py is also included in this package that gives an example of how to launch the status_led_node.

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='status_led_pkg',
                namespace='status_led_pkg',
                executable='status_led_node',
                name='status_led_node'
            )
        ])

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|led_solid|SetStatusLedSolidSrv|A service that is called to stop the current effect on the LED light passed as parameter and set a solid light effect with the color passed as parameter.|
|led_blink|SetStatusLedBlinkSrv|A service that is called to stop the current effect on the LED light passed as parameter and start a blink light effect with the colors passed as parameter.|

