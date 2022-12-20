# ---------------------------------------------------------------------------- #
#                                    Imports                                   #
# ---------------------------------------------------------------------------- #

# ------------------------- System related Libraries ------------------------- #
import os
import sys
import argparse
import yaml

# ----------------------------- Common Libraries ----------------------------- #
import numpy as np
import cv2

# --------------------------- ROS related Libraries -------------------------- #
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from cv_bridge import CvBridge


# ---------------------------------------------------------------------------- #
#                                   Functions                                  #
# ---------------------------------------------------------------------------- #

# ------------------------ Check if directory is valid ----------------------- #
def dir_path(string):
    if os.path.isdir(string):
        return string
    else:
        raise NotADirectoryError(string)

# ----------------------- Check if file path is valid ----------------------- #
def file_path(string):
    if os.path.isfile(string):
        return string
    else:
        raise FileNotFoundError(string)

# ------------------------------ Undistort Image ----------------------------- #
def undistort(input, distortion_data):
    
    # Extract Camera Matrix
    camera_matrix = np.array(distortion_data['camera_matrix']['data'])
    camera_matrix = np.reshape(camera_matrix, (distortion_data['camera_matrix']['rows'], distortion_data['camera_matrix']['cols']))
    
    # Extract Distortion Matrix
    distortion_matrix = np.array(distortion_data['distortion_coefficients']['data'])

    # Undistort and Return
    return cv2.undistort(input, camera_matrix, distortion_matrix)

# ---------------------------------------------------------------------------- #
#                           Setup & Argument Handling                          #
# ---------------------------------------------------------------------------- #
arg_parser  = argparse.ArgumentParser(description='Extracts Images from ROS2 Bags')

# ------------------------------- Add Arguments ------------------------------ #
arg_parser.add_argument('rosbag_file_path', help='Path to rosbag to extract the data from', type=dir_path)
arg_parser.add_argument('output_dir', help='Path to directory where extracted data should be stored', type=dir_path)
arg_parser.add_argument('-u', "--undistort", action="store_true")
arg_parser.add_argument('-p', '--camera_info_path', help="Path to folder containing yaml config files for camera info for all cameras", type=dir_path)
arg_parser.add_argument('-v', "--verbose", action="store_true")

# ------------------------------ Parse Arguments ----------------------------- #
args = arg_parser.parse_args()

OUTPUT_DIR  = args.output_dir
ROSBAG_FILE = args.rosbag_file_path

if args.undistort:
    dir_path(args.camera_info_path)
    distortion_dict = dict() 

# --------------------------- Create ROS-CV Bridge --------------------------- #
bridge = CvBridge()

# ---------------------------------------------------------------------------- #
#                                  Main Script                                 #
# ---------------------------------------------------------------------------- #

# ---------------- Create reader instance and open for reading --------------- #
with Reader(ROSBAG_FILE) as reader:

    camera_found = False

    for connection in reader.connections:
        if args.verbose:
            print(connection.topic," : ", connection.msgtype)
        if 'camera' in connection.topic:
            camera_found = True

    if camera_found is False:
        exit()

    # Iterator dictionary to go over camera messages
    iterator = {
        '/camera/rear_left/image/compressed'            : 0,
        '/camera/rear_right/image/compressed'           : 0,
        '/camera/front_left/image/compressed'           : 0,
        '/camera/front_left_center/image/compressed'    : 0,
        '/camera/front_right_center/image/compressed'   : 0,
        '/camera/front_right/image/compressed'          : 0,
        '/vimba_rear_left/image'            : 0,
        '/vimba_rear_right/image'           : 0,
        '/vimba_front_left/image'           : 0,
        '/vimba_front_left_center/image'    : 0,
        '/vimba_front_right_center/image'   : 0,
        '/vimba_front_right/image'          : 0,
    }

    for connection, timestamp, rawdata in reader.messages():
        if connection.topic in iterator.keys():
            
            # Update iterator for this topic
            iterator[connection.topic] += 1

            # Extract message from rosbag
            msg = deserialize_cdr(rawdata, connection.msgtype)
            output_topic = None
            if (connection.msgtype == "sensor_msgs/msg/CompressedImage"):
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv2_msg = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                output_topic = connection.topic[7:-16]
            else:
                # Convert to cv2 image
                cv2_msg = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                output_topic = connection.topic[:-5]

            # Create a directory for topic in output dir if it does not exist
            output_directory = OUTPUT_DIR + output_topic
            if not os.path.exists(output_directory):
                if args.verbose:
                    print("Creating Directory: ", output_directory)
                os.mkdir(output_directory)

            output_file_path = output_directory + 'Image' + '_' + '{0:010d}'.format(iterator[connection.topic]) + '_' + str(msg.header.stamp.sec) + '_' + str(msg.header.stamp.nanosec) + '.jpg'
            
            # Undistort Image before Saving
            if args.undistort:
                # If distortion parameters not loaded then load them once
                if connection.topic[1:-6] not in distortion_dict:
                    yaml_file_path = args.camera_info_path + connection.topic[1:-6] + '.yaml'
                    distortion_fp = open(file_path(yaml_file_path))
                    distortion_dict[connection.topic[1:-6]] = yaml.safe_load(distortion_fp)
                cv2_msg = undistort(cv2_msg, distortion_dict[connection.topic[1:-6]])

            # Save Image
            if args.verbose:
                print('Saving ' + output_file_path)

            if not cv2.imwrite(output_file_path, cv2_msg):
                raise Exception("Could not write image")
            