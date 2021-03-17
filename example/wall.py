#!/usr/bin/env python3
# 2021/03/17 hato #!/usr/bin/env python -> !/usr/bin/env python3

# Python includes
import numpy as np
import random

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()

import rviz_tools_py as rviz_tools


# Initialize the ROS Node
rospy.init_node('wall', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
  # 2021/03/16 hato ------------------------------ start ------------------------------
  # print "Shutting down node"
  print("Shutting down node")
  # 2021/03/16 hato ------------------------------  end  ------------------------------
  markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('/world', 'wall_marker')

Config={
  "anchors":[0,0,1000,1000],
  "height":1000
}

while not rospy.is_shutdown():
  try:
    Config.update(rospy.get_param("/config/wall"))
  except Exception as e:
    # 2021/03/16 hato ------------------------------ start ------------------------------
    # print "get_param exception in wall.py:",e.args
    print("get_param exception in wall.py:",e.args)
    # 2021/03/16 hato ------------------------------  end  ------------------------------
  polygon = Polygon()
  xys=np.asarray(Config["anchors"]).reshape((-1,2))
  h=Config["height"]
  for xy in xys:
    polygon.points.append( Point(xy[0],xy[1],h) )
    polygon.points.append( Point(xy[0],xy[1],0) )
    polygon.points.append( Point(xy[0],xy[1],h) )

  markers.publishPolygon(polygon, 'yellow', 20, 5.0) # path, color, width, lifetime

  rospy.Rate(1).sleep() #1 Hz
