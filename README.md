# openhab_bridge_publisher
Publishes commands to openhab using a bridge between openHAB and ROS with HABApp

## Installation

Go to your src folder of your catkin_ws and clone the repository:

```
cd ~/catkin_ws/src
git clone https://github.com/Michdo93/openhab_bridge_publisher.git
cd ~/catkin_ws
catkin_make
```

## Usage

You can run each subscriber like following:

```
rosrun openhab_bridge_publisher ColorPublisher.py
rosrun openhab_bridge_publisher ContactPublisher.py
rosrun openhab_bridge_publisher DateTimePublisher.py
rosrun openhab_bridge_publisher DimmerPublisher.py
rosrun openhab_bridge_publisher ImagePublisher.py
rosrun openhab_bridge_publisher LocationPublisher.py
rosrun openhab_bridge_publisher NumberPublisher.py
rosrun openhab_bridge_publisher RollershutterPublisher.py
rosrun openhab_bridge_publisher StringPublisher.py
rosrun openhab_bridge_publisher SwitchPublisher.py
```
