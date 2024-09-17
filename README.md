## Install RosBridge:
1. Install RosBridge server by running the following command:
    ```shell
    sudo apt install ros-<ROS_DISTRO>-rosbridge-server
    ```
2. Source your ROS distribution setup:
    ```shell
    source /opt/ros/<ROS_DISTRO>/setup.bash
    ```
3. Install script dependencies
    ```shell
    sudo apt install expect -y
    ```

## Launch RosBridge:
1. Launch the RosBridge server with the following command:
    ```shell
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
    ```
   This will start the server on port 9090.


## Setup Raspberry Pi
1. Connecting Raspberry Pi to L298N:
    - IN1/IN2 (left motor) should connect to GPIO 17 and GPIO 27.
    - IN3/IN4 (right motor) should connect to GPIO 23 and GPIO 24.
```bash
sudo apt install python3-rpi.gpio
```




Roadmap

1. js
2. asyncronous programming
3. 