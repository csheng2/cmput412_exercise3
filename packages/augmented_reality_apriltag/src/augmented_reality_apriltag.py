#!/usr/bin/env python3
from renderClass import Renderer
import rospy
import yaml
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import Header, ColorRGBA
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import CompressedImage, CameraInfo
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
    self.veh_name = rospy.get_namespace().strip("/")

    # Get static parameters
    extrinsic_calibration_file = f'/data/config/calibrations/camera_extrinsic/{self.veh_name}.yaml'
    intrinsic_calibration_file = f'/data/config/calibrations/camera_intrinsic/{self.veh_name}.yaml'
    self.homography = self.readYamlFile(extrinsic_calibration_file)['homography']
    self.intrinstic = self.readYamlFile(intrinsic_calibration_file)
    self.camera_info_msg = rospy.wait_for_message(f'/{self.veh_name}/camera_node/camera_info', CameraInfo)

    rospack = rospkg.RosPack()
    # Initialize an instance of Renderer giving the model in input.
    self.renderer = Renderer(rospack.get_path('augmented_reality_apriltag') + '/src/models/duckie.obj')
    self.bridge = CvBridge()

    # Initialize LED color-changing
    self.color_publisher = rospy.Publisher(f'/{self.veh_name}/led_emitter_node/led_pattern', LEDPattern, queue_size = 1)
    self.pattern = LEDPattern()
    self.pattern.header = Header()
    self.colors = {
      'purple': {'r': 1.0, 'g': 0.0, 'b': 1.0, 'a': 1.0},
      'blue': {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 1.0},
      'cyan': {'r': 0.0, 'g': 1.0, 'b': 1.0, 'a': 1.0},
      'red': {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0},
      'green': {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0},
      'yellow': {'r': 1.0, 'g': 1.0, 'b': 0.0, 'a': 1.0},
      'white': {'r': 1.0, 'g': 1.0, 'b': 1.0, 'a': 1.0},
      'off': {'r': 0.0, 'g': 0.0, 'b': 0.0, 'a': 0.0}
    }


  def projection_matrix(self):
    """
      Write here the compuatation for the projection matrix, namely the matrix
      that maps the camera reference frame to the AprilTag reference frame.
    """
    projection_matrix = []
    data = self.intrinstic['projection_matrix']['data']
    cols = self.intrinstic['projection_matrix']['cols']
    rows = self.intrinstic['projection_matrix']['rows']

    for r in range(0, rows):
      row = []
      for c in range(0, cols):
        row.append(data[r*cols + c])      
      projection_matrix.append(row)
    
    return projection_matrix

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

  def change_color(self, color):
    '''
    Code for this function was inspired by 
    "duckietown/dt-core", file "led_emitter_node.py"
    Link: https://github.com/duckietown/dt-core/blob/daffy/packages/led_emitter/src/led_emitter_node.py
    Author: GitHub user liampaull
    '''
    if color not in self.colors:
      color = 'white' # default color

    self.pattern.header.stamp = rospy.Time.now()
    rgba = ColorRGBA()
    rgba.r = self.colors[color]['r']
    rgba.g = self.colors[color]['g']
    rgba.b = self.colors[color]['b']
    rgba.a = self.colors[color]['a']
    self.pattern.rgb_vals = [rgba] * 5
    self.color_publisher.publish(self.pattern)


if __name__ == '__main__':
  # Initialize the node
  camera_node = ARNode(node_name='augmented_reality_apriltag_node')
  # Keep it spinning to keep the node alive
  rospy.spin()