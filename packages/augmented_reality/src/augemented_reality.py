#!/usr/bin/env python3

import yaml
import rospy
from duckietown.dtros import DTROS, NodeType

'''
Basic code for a node was taken from
Unit C-2: Development in the Duckietown infrastructure, Hands-on Robotics Development using Duckietown
Link: https://docs.duckietown.org/daffy/duckietown-robotics-development/out/dt_infrastructure.html
'''
class AugmentedRealityNode(DTROS):
  def __init__(self, node_name):
    # initialize the DTROS parent class
    super(AugmentedRealityNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

  def readYamlFile(self,fname):
    """
    Reads the YAML file in the path specified by 'fname'.
    E.G. :
        the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
    """
    with open(fname, 'r') as in_file:
      try:
        yaml_dict = yaml.load(in_file)
        return yaml_dict
      except yaml.YAMLError as exc:
        rospy.loginfo("YAML syntax error. File: %s fname. Exc: %s"
          %(fname, exc), type='fatal')
        rospy.signal_shutdown()
        return

if __name__ == '__main__':
  # create the node
  node = AugmentedRealityNode(node_name='augmented_reality_node')
  # keep spinning
  rospy.spin()