# AWS DeepRacer status LED package

## Overview

The AWS DeepRacer status LED ROS package creates the `status_led_node`, which is part of the core AWS DeepRacer application and launches from the `deepracer_launcher`. For more information about the application and the components, see the [aws-deepracer-launcher repository](https://github.com/aws-deepracer/aws-deepracer-launcher).

This node contains the logic for the blink and solid light effects for the status LED lights on the side of the AWS DeepRacer device. It provides services and functions that enable or disable the RGB GPIO ports for the LED lights and start and stop the particular effects on the LED lights.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation
Follow these steps to install the AWS DeepRacer status LED package.

### Prerequisites

The AWSDeepRacer device comes with all the prerequisite packages and libraries installed to run the `status_led_pkg`. For more information about the preinstalled set of packages and libraries on the AWSDeepRacer and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `status_led_pkg` specifically depends on the following ROS 2 packages as build and run dependencies.

* `deepracer_interfaces_pkg`: This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the `status_led_pkg` on the AWS DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-status-led-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-status-led-pkg
        rosws update

1. Resolve the `status_led_pkg` dependencies:

        cd ~/deepracer_ws/aws-deepracer-status-led-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `status_led_pkg` and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-status-led-pkg && colcon build --packages-select status_led_pkg deepracer_interfaces_pkg

## Usage

Although the `status_led_node` is built to work with the AWS DeepRacer application, you can run it independently for development, testing, and debugging purposes.

### Run the node

To launch the built `status_led_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-status-led-pkg/install/setup.bash 

1. Launch the `status_led_node` using the launch script:

        ros2 launch status_led_pkg status_led_pkg_launch.py

## Launch files

The `status_led_pkg_launch.py`, included in this package, provides an example demonstrating how to launch the `status_led_node`.

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

### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`led_solid`|`SetStatusLedSolidSrv`|A service that is called to stop the current effect on the LED light passed as a parameter and set a solid light effect with the color passed as a parameter.|
|`led_blink`|`SetStatusLedBlinkSrv`|A service that is called to stop the current effect on the LED light passed as a parameter and start a blink light effect with the colors passed as a parameter.|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)

