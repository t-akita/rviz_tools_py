#!/usr/bin/env python

# Python includes
import numpy as np
import copy

# ROS includes
import roslib
import rospy
from std_msgs.msg import ColorRGBA
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

markers = rviz_tools.RvizMarkers('camera/master0', 'cutter_marker')

Param={
  "refs":"/cropper/master",
  "base":-100,
  "offset":200,
  "width":10,
  "radius":75,
  "distance":550,
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
  tr1=Transform()
  tr1.rotation.w=np.sqrt(2)/2
  tr1.rotation.y=np.sqrt(2)/2
  tr1.translation.x=Param["base"] if "base" not in Refs else Refs["base"]
  tr1.translation.z=Param["distance"] if "distance" not in Refs else Refs["distance"]
  radius=Param["radius"] if "radius" not in Refs else Refs["radius"]
  width=Param["width"] if "width" not in Refs else Refs["width"]
  T1=tflib.toRT(tr1)
  color='translucent_dark'
  markers.publishCylinder(T1,color, width, radius, 1.0) # pose, color, scale, lifetime
  tr2=copy.copy(tr1)
  offset=Param["offset"] if "offset" not in Refs else Refs["offset"]
  tr2.translation.x=tr2.translation.x+offset
  T2=tflib.toRT(tr2)
  markers.publishCylinder(T2,color, width, radius, 1.0)
