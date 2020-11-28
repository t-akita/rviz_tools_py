#!/usr/bin/env python

# Python includes
import numpy as np
import copy

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
  "refs":"/cropper/master",
  "depth":200,
  "length":300,
  "distance":400,
  "color":"white"
}
Refs={}
while not rospy.is_shutdown():
  rospy.Rate(1).sleep() #1 Hz
  try:
    Param.update(rospy.get_param("/cutter"))
  except Exception as e:
    pass
  try:
    Refs=rospy.get_param(Param["refs"])
  except Exception as e:
    pass
  if "base" not in Refs: continue
  if "offset" not in Refs: continue
# Publish a plane using a ROS Pose Msg
  tr1=Transform()
  tr1.translation.x=Refs["base"]
  tr1.translation.z=Param["distance"]+Param["depth"]/2
  tr1.rotation.w=np.sqrt(2)/2
  tr1.rotation.y=np.sqrt(2)/2
  T1=tflib.toRT(tr1)
  markers.publishPlane(T1,Param["depth"],Param["length"],Param["color"], 1.0) # pose, depth, width, color, lifetime
  tr2=copy.copy(tr1)
  tr2.translation.x=Refs["base"]+Refs["offset"]
  T2=tflib.toRT(tr2)
  markers.publishPlane(T2,Param["depth"],Param["length"],Param["color"], 1.0) # pose, depth, width, color, lifetime
