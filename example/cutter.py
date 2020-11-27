#!/usr/bin/env python

# Python includes
import numpy as np
import random

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon, Transform
from tf import transformations # rotation_matrix(), concatenate_matrices()
from rovi_utils import tflib
import rviz_tools_py as rviz_tools


# Initialize the ROS Node
rospy.init_node('cutter', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
  print "Shutting down node"
  markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('/camera/master0', 'cutter_marker')

Param={
  "offset":0,
  "length":100,
  "distance":500,
  "color":"white"
}

while not rospy.is_shutdown():
  try:
    Param.update(rospy.get_param("/cutter"))
  except Exception as e:
    print "get_param exception in wall.py:",e.args
  # Publish a plane using a ROS Pose Msg
  tr=Transform()
  tr.translation.x=Param["offset"]
  tr.translation.z=Param["distance"]
  tr.rotation.w=np.sqrt(2)/2
  tr.rotation.y=np.sqrt(2)/2
  T=tflib.toRT(tr)
  markers.publishPlane(T,Param["length"],Param["length"],Param["color"], 1.0) # pose, depth, width, color, lifetime
  rospy.Rate(1).sleep() #1 Hz
