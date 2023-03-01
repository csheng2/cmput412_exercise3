#!/usr/bin/env python3
import rospy, sys, tf

from tf2_ros import StaticTransformBroadcaster
from duckietown.dtros import DTROS, NodeType
from geometry_msgs.msg import TransformStamped

class StaticTF2BroadcasterNode(DTROS):
  def __init__(self, node_name):
    super(StaticTF2BroadcasterNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
    self.node_name = node_name

    if len(sys.argv) < 8:
      rospy.logerr('Invalid number of parameters\nusage: '
                    './static_turtle_tf2_broadcaster.py '
                    'child_frame_name x y z roll pitch yaw')
      sys.exit(0)
    else:
      if sys.argv[1] == 'world':
        rospy.logerr('Your static turtle name cannot be "world"')
        sys.exit(0)

      rospy.init_node('my_static_tf2_broadcaster')
      broadcaster = StaticTransformBroadcaster()
      static_transformStamped = TransformStamped()

      static_transformStamped.header.stamp = rospy.Time.now()
      static_transformStamped.header.frame_id = "world"
      static_transformStamped.child_frame_id = sys.argv[1]

      static_transformStamped.transform.translation.x = float(sys.argv[2])
      static_transformStamped.transform.translation.y = float(sys.argv[3])
      static_transformStamped.transform.translation.z = float(sys.argv[4])

      quat = tf.transformations.quaternion_from_euler(
                  float(sys.argv[5]),float(sys.argv[6]),float(sys.argv[7]))
      static_transformStamped.transform.rotation.x = quat[0]
      static_transformStamped.transform.rotation.y = quat[1]
      static_transformStamped.transform.rotation.z = quat[2]
      static_transformStamped.transform.rotation.w = quat[3]

      broadcaster.sendTransform(static_transformStamped)

if __name__ == '__main__':
  # create node
  node = StaticTF2BroadcasterNode("static_tf2_broadcaster_node")
  rospy.spin()
  # ---
  rospy.signal_shutdown("done")
