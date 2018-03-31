full_path_to_images = "/home/peteflo/spartan/src/tsdf-fusion/spartan_data/fusion1519825352.8/images"

import os

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
with open(camera_intrinsics_out, 'a') as the_file:
    the_file.write(" "+sci(K_matrix[0])+"   "+sci(K_matrix[1])+"   "+sci(K_matrix[2])+"  \n")
    the_file.write(" "+sci(K_matrix[3])+"   "+sci(K_matrix[4])+"   "+sci(K_matrix[5])+"  \n")
    the_file.write(" "+sci(K_matrix[6])+"   "+sci(K_matrix[7])+"   "+sci(K_matrix[8])+"  \n") 
