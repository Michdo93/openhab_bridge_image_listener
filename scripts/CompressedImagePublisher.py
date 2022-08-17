#!/usr/bin/python
import os
import rospy
from openhab_msgs.msg import ImageCommand
import argparse
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# OpenCV
import cv2

VERBOSE = False


class CompressedImagePublisher(object):
    """Node example class."""

    def __init__(self, item_name, topic):
        self.topic = topic
        self.item_name = item_name
        self.pub = rospy.Publisher(
            "/openhab/items/" + self.item_name + "/command", ImageCommand, queue_size=10)
        self.sub = rospy.Subscriber(self.topic, CompressedImage, self.callback)
        self.rate = rospy.Rate(10)  # 10hz

        # Initialize message variables.
        self.enable = False
        self.message = None
        self.bridge = CvBridge()
        self.image = None

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        """Turn on publisher."""
        self.enable = True
        self.pub = rospy.Publisher(
            "/openhab/items/" + self.item_name + "/command", ImageCommand, queue_size=10)
        self.sub = rospy.Subscriber(self.topic, CompressedImage, self.callback)

    def callback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.image = data

        np_arr = np.fromstring(self.image.data, np.uint8)
        cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        try:
            self.image = self.bridge.cv2_to_imgmsg(cv2_img, 'bgr8')
        except CvBridgeError as e:
            print(e)

        self.message = ImageCommand()
        self.message.isnull = False
        self.message.command = self.image
        self.message.header.stamp = rospy.Time.now()
        self.message.header.frame_id = "/base_link"
        self.message.item = self.item_name

        message = "Publishing %s to %s at %s" % (
            self.topic, self.message.item, rospy.get_time())
        rospy.loginfo(message)

        self.pub.publish(self.message)

    def stop(self):
        """Turn off publisher."""
        self.enable = False
        self.pub.unregister()
        self.sub.unregister()


# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("ImageListerPublisherNode", anonymous=True)
    # Go to class functions that do all the heavy lifting.

    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", type=str, required=True,
                        help="Specify a topic with sensor_msgs/CompressedImage. This topic will be subscribed. The subscribed compressed image will be published to a given openHAB item.")
    args = parser.parse_args()

    topic = str(args.topic)

    compressedImagePublisher = CompressedImagePublisher("testImage", topic)

    try:
        compressedImagePublisher.start()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
