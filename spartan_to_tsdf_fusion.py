full_path_to_images = "/home/peteflo/spartan/src/tsdf-fusion/spartan_data/fusion1519825352.8/images"

import os



### HANDLE INTRINSICS

camera_info_yaml = os.path.join(full_path_to_images, "camera_info.yaml")

import yaml
with open(camera_info_yaml, 'r') as stream:
    try:
        camera_info_dict = yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)

K_matrix = camera_info_dict['camera_matrix']['data']
print K_matrix
n = K_matrix[0]

def sci(n):
  return "{:.8e}".format(n)

spartan_data_path = os.path.dirname(os.path.dirname(full_path_to_images))
camera_intrinsics_out = os.path.join(spartan_data_path,"camera-intrinsics.txt")
with open(camera_intrinsics_out, 'w') as the_file:
    the_file.write(" "+sci(K_matrix[0])+"	 "+sci(K_matrix[1])+"	 "+sci(K_matrix[2])+"	\n")
    the_file.write(" "+sci(K_matrix[3])+"	 "+sci(K_matrix[4])+"	 "+sci(K_matrix[5])+"	\n")
    the_file.write(" "+sci(K_matrix[6])+"	 "+sci(K_matrix[7])+"	 "+sci(K_matrix[8])+"	\n") 


### HANDLE POSES

pose_data_yaml = os.path.join(full_path_to_images, "pose_data.yaml")
with open(pose_data_yaml, 'r') as stream:
    try:
        pose_data_dict = yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)

print pose_data_dict[0]

import numpy as np
import math

# this function cowbody copied from:
# https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
def quaternion_matrix(quaternion):
    _EPS = np.finfo(float).eps * 4.0
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

def spartan_pose_to_homogeneous_transform(spartan_pose):
    quat_w = spartan_pose['camera_to_world']['quaternion']['w']
    quat_x = spartan_pose['camera_to_world']['quaternion']['x']
    quat_y = spartan_pose['camera_to_world']['quaternion']['y']
    quat_z = spartan_pose['camera_to_world']['quaternion']['z']
    homogeneous_transform = quaternion_matrix([quat_w, quat_x, quat_y, quat_z])
    homogeneous_transform[0,3] = spartan_pose['camera_to_world']['translation']['x']
    homogeneous_transform[1,3] = spartan_pose['camera_to_world']['translation']['y']
    homogeneous_transform[2,3] = spartan_pose['camera_to_world']['translation']['z']
    return homogeneous_transform

for i in pose_data_dict:
    print i
    print pose_data_dict[i]
    pose4 = spartan_pose_to_homogeneous_transform(pose_data_dict[i])
    depth_image_filename = pose_data_dict[i]['depth_image_filename']
    prefix = depth_image_filename.split("depth")[0]
    print prefix
    pose_file_name = prefix+"pose.txt"
    pose_file_full_path = os.path.join(full_path_to_images,pose_file_name)
    with open(pose_file_full_path, 'w') as the_file:
        the_file.write(" "+sci(pose4[0,0])+"	 "+sci(pose4[0,1])+"	 "+sci(pose4[0,2])+"	 "+sci(pose4[0,3])+"	\n")
        the_file.write(" "+sci(pose4[1,0])+"	 "+sci(pose4[1,1])+"	 "+sci(pose4[1,2])+"	 "+sci(pose4[1,3])+"	\n")
        the_file.write(" "+sci(pose4[2,0])+"	 "+sci(pose4[2,1])+"	 "+sci(pose4[2,2])+"	 "+sci(pose4[2,3])+"	\n")
        the_file.write(" "+sci(pose4[3,0])+"	 "+sci(pose4[3,1])+"	 "+sci(pose4[3,2])+"	 "+sci(pose4[3,3])+"	\n")

quit()

