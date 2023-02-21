#!/usr/bin/env python3
from renderClass import Renderer

import rospy
import yaml
from duckietown.dtros import DTROS, NodeType
from cv_bridge import CvBridgeError

import rospkg 


"""
  Template code was taken from
  Github, "cra1-template", author github user viciopoli01
  Link: https://github.com/duckietown-ethz/cra1-template/blob/master/packages/augmented_reality_apriltag/src/augmented_reality_apriltag.py
"""

class ARNode(DTROS):

  def __init__(self, node_name):

    # Initialize the DTROS parent class
    super(ARNode, self).__init__(node_name=node_name,node_type=NodeType.GENERIC)
    self.veh = rospy.get_namespace().strip("/")

    rospack = rospkg.RosPack()
    # Initialize an instance of Renderer giving the model in input.
    self.renderer = Renderer(rospack.get_path('augmented_reality_apriltag') + '/src/models/duckie.obj')

    #
    #   Write your code here
    #

    
  def projection_matrix(self, intrinsic, homography):
    """
      Write here the compuatation for the projection matrix, namely the matrix
      that maps the camera reference frame to the AprilTag reference frame.
    """

    #
    # Write your code here
    #

  def readImage(self, msg_image):
    """
      Convert images to OpenCV images
      Args:
        msg_image (:obj:`CompressedImage`) the image from the camera node
      Returns:
        OpenCV image
    """
    try:
      cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_image)
      return cv_image
    except CvBridgeError as e:
      self.log(e)
      return []

  def readYamlFile(self,fname):
    """
      Reads the 'fname' yaml file and returns a dictionary with its input.

      You will find the calibration files you need in:
      `/data/config/calibrations/`
    """
    with open(fname, 'r') as in_file:
      try:
        yaml_dict = yaml.load(in_file)
        return yaml_dict
      except yaml.YAMLError as exc:
        self.log("YAML syntax error. File: %s fname. Exc: %s"
          %(fname, exc), type='fatal')
        rospy.signal_shutdown()
        return


  def onShutdown(self):
    super(ARNode, self).onShutdown()


if __name__ == '__main__':
  # Initialize the node
  camera_node = ARNode(node_name='augmented_reality_apriltag_node')
  # Keep it spinning to keep the node alive
  rospy.spin()