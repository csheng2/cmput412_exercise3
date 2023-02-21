#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import os

class AprilTagNode(DTROS):
  def __init__(self, node_name):
    # initialize the DTROS parent class
    super(AprilTagNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
    self.camera_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed", CompressedImage, self.callback)


if __name__ == '__main__':
  # create the node
  node = AprilTagNode(node_name='apriltag_node')
  # keep spinning
  rospy.spin()