from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix

import math


import numpy as np



#A class of helper functions blatantly stolen from Paul Ruvolo, our hero and savior 


def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] 
    """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

def convert_pose_to_xy_and_theta(pose):
    """ pose: geometry_msgs.Pose object
        returns tuple of form (x, y, yaw)
    """
    orientation_tuple = (pose.pose.orientation.x,
                         pose.pose.orientation.y,
                         pose.pose.orientation.z,
                         pose.pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.pose.position.x, pose.pose.position.y, angles[2]

def fix_map_to_odom_transform(self, stamp):
  """ This method constantly updates the offset of the map and 
      odometry coordinate systems based on the latest results from
      the localizer 
  """
  p = PoseStamped(header=Header(stamp=stamp,
                              frame_id="base_laser_link"),
                pose=Pose())
  odom_pose = TransformListener().transformPose("odom", p)

  tf_listener = TransformListener()
  (translation, rotation) = convert_pose_inverse_transform(odom_pose)
  pose = convert_translation_rotation_to_pose(translation,rotation)
  p = PoseStamped(pose=pose,
                  header=Header(stamp=stamp,frame_id="base_laser_link"))
  odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
  return convert_pose_inverse_transform(odom_to_map.pose)

def convert_translation_rotation_to_pose(translation, rotation):
  """ Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
  return Pose(position=Point(x=translation[0],y=translation[1],z=translation[2]), orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3]))

def convert_pose_inverse_transform(pose):
  """ Helper method to invert a transform (this is built into the tf C++ classes, but ommitted from Python) """
  translation = np.zeros((4,1))
  translation[0] = -pose.position.x
  translation[1] = -pose.position.y
  translation[2] = -pose.position.z
  translation[3] = 1.0

  rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
  euler_angle = euler_from_quaternion(rotation)
  rotation = np.transpose(rotation_matrix(euler_angle[2], [0,0,1]))       # the angle is a yaw
  transformed_translation = rotation.dot(translation)

  translation = (transformed_translation[0], transformed_translation[1], transformed_translation[2])
  rotation = quaternion_from_matrix(rotation)
  return (translation, rotation)