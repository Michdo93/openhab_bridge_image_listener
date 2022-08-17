# openhab_bridge_image_listener

ROS package which listens and subscribes to the `sensor_msgs/Image` or `sensor_msgs/CompressedImage` topics. This image will be published as command to openHAB using a bridge between openHAB and ROS with HABApp. 

## Installation

Go to your src folder of your catkin_ws and clone the repository:

```
cd ~/catkin_ws/src
git clone https://github.com/Michdo93/openhab_bridge_image_listener.git
cd ~/catkin_ws
catkin_make
```

## Usage

You can run each subscriber like following:

```
rosrun openhab_bridge_image_listener CompressedImagePublisher.py --topic <topic>
rosrun openhab_bridge_image_listener ImagePublisher.py --topic <topic>
```

Please replace `<topic>` with a topic where you can subscribe a `sensor_msgs/Image` or `sensor_msgs/CompressedImage`.
