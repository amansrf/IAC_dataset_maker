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
from cv_bridge import CvBridge
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message



# ---------------------------------------------------------------------------- #
#                                   Functions                                  #
# ---------------------------------------------------------------------------- #

# ------------------------ Check if directory is valid ----------------------- #
def dir_path(string):
    if os.path.isdir(string):
        print(f"Is a directory! {string}")
        return string
    else:
        print(f"Is not a directory! {string}")
        raise NotADirectoryError(string)

# ----------------------- Check if file path is valid ----------------------- #
def file_path(string):
    if os.path.isfile(string):
        print(f"Is a file! {string}")
        return string
    else:
        print(f"Is not a string! {string}")
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
arg_parser.add_argument('-c', "--compressed", action="store_true")
arg_parser.add_argument('-p', '--camera_info_path', help="Path to folder containing yaml config files for camera info for all cameras", type=dir_path)
arg_parser.add_argument('-v', "--verbose", action="store_true")

# ------------------------------ Parse Arguments ----------------------------- #
args = arg_parser.parse_args()

OUTPUT_DIR  = args.output_dir
print("Output Directory: ", OUTPUT_DIR)
ROSBAG_FILE_PATH = args.rosbag_file_path

if args.undistort:
    dir_path(args.camera_info_path)
    distortion_dict = dict() 

# --------------------------- Create ROS-CV Bridge --------------------------- #
bridge = CvBridge()

# ---------------------------------------------------------------------------- #
#                                  Main Script                                 #
# ---------------------------------------------------------------------------- #

# ---------------- Create reader instance and open for reading --------------- #
# with Reader(ROSBAG_FILE) as reader:
# Check if the extension is a db3 or mcap
files = os.listdir(ROSBAG_FILE_PATH)
for file in files:
    if file.endswith(".db3"):
        store_type = "sqlite3"
        print("Detected Input bag is a db3 file.")
    elif file.endswith(".mcap"):
        store_type = "mcap"
        print("Detected Input bag is a mcap file.")
if not store_type:
    print("Error: Input bag is not a db3 or mcap file.")
    exit()

reader = rosbag2_py.SequentialReader()

# ----------------------------- OBTAIN ALL TOPICS ---------------------------- #
# Opens the bag files and sets the converter options
try:
    reader.open(
        rosbag2_py.StorageOptions(uri=ROSBAG_FILE_PATH, storage_id=store_type),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
except Exception as e:
    print(e)
    exit()

camera_found = False

# Iterator dictionary to go over camera messages
if args.compressed:
    iterator = {
        '/vimba_front_left_center/image/compressed'  :0,
        '/vimba_front_right_center/image/compressed' :0,
        '/vimba_front_left/image/compressed'         :0,
        '/vimba_front_right/image/compressed'        :0,
        '/vimba_rear_left/image/compressed'          :0,
        '/vimba_rear_right/image/compressed'         :0,
    }
else:
    iterator = {
        '/vimba_rear_left/image'            : 0,
        '/vimba_rear_right/image'           : 0,
        '/vimba_front_left/image'           : 0,
        '/vimba_front_left_center/image'    : 0,
        '/vimba_front_right_center/image'   : 0,
        '/vimba_front_right/image'          : 0,
    }

# Checking if all the topics that are going to be collected are in the bag
TOPIC_TYPES = reader.get_all_topics_and_types()
TYPE_MAP = {TOPIC_TYPES[i].name: TOPIC_TYPES[i].type for i in range(len(TOPIC_TYPES))}
if iterator:
    for topic in iterator.keys():
        if topic not in TYPE_MAP:
            print("ERROR: The topic ", topic, " is not in the input bag.")
            # exit()
        print("Topic ", topic, " found in the input bag.")
else:
    print("FATAL ERROR: TOPICS_TO_EXTRACT not defined.")

counter = 0

while reader.has_next():
    
    # Read the next message
    topic_name, data, timestamp = reader.read_next()
    
    if topic_name in iterator.keys():
        # Update iterator for this topic
        iterator[topic_name] += 1

        # Extract message from rosbag
        msg_type = TYPE_MAP[topic_name]
        msg_ser = get_message(msg_type)
        msg = deserialize_message(data, msg_ser)
        output_topic = None
        if (args.compressed and msg_type == "sensor_msgs/msg/CompressedImage"):
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv2_msg = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            output_topic = topic_name[7:-17]
        else:
            # Convert to cv2 image
            cv2_msg = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            output_topic = topic_name[1:-6]

        # Create a directory for topic in output dir if it does not exist
        # print("Output Topic: ", output_topic)
        output_directory = os.path.join(OUTPUT_DIR, output_topic)
        
        if not os.path.exists(output_directory):
            if args.verbose:
                print("Creating Directory: ", output_directory)
            os.mkdir(output_directory)

        output_file_path = os.path.join(output_directory, 'Image' + '_' + '{0:010d}'.format(iterator[topic_name]) + '_' + str(msg.header.stamp.sec) + '_' + str(msg.header.stamp.nanosec) + '.jpg')

        # Undistort Image before Saving
        if args.undistort:
            # If distortion parameters not loaded then load them once
            if topic_name[1:-6] not in distortion_dict:
                yaml_file_path = args.camera_info_path + topic_name[1:-6] + '.yaml'
                distortion_fp = open(file_path(yaml_file_path))
                distortion_dict[topic_name[1:-6]] = yaml.safe_load(distortion_fp)
            cv2_msg = undistort(cv2_msg, distortion_dict[topic_name[1:-6]])

        # Save Image
        

        if args.verbose:
            print('Saving ' + output_file_path)
        counter += 1
        if counter %1000 == 0:
            print(f"Processed {counter} Images")

        if not cv2.imwrite(output_file_path, cv2_msg):
            raise Exception("Could not write image")
        
# Close the bag file
del reader
