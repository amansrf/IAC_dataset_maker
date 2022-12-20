#!/bin/sh

# ---------------------------------------------------------------------------- #
#                               DEFAULT VARIABLES                              #
# ---------------------------------------------------------------------------- #
DATA_DIR_DEFAULT="/media/Public/ROSBAG_BACKUPS/rosbag2_2022_11_09-16_27_25"
VERBOSE_DEFAULT=1
UNDISTORT_DEFAULT=0
CALIB_DIR_DEFAULT="/home/roar/ART/perception/Camera/Calibration_new/"
OUTPUT_BASE_DIR_DEFAULT="/media/Public/cam_tms/"

# ---------------------------------------------------------------------------- #
#                          PARSE ENVIRONMENT VARIABLES                         #
# ---------------------------------------------------------------------------- #

# --------------------------- ROSBAG DATA DIRECTORY -------------------------- #
if [ -z ${DATA_DIR+x} ]; then
    echo "----------------------------------------------------------------------------"
    echo "                         DATA_DIR not defined.                              "
    DATA_DIR=$DATA_DIR_DEFAULT
    echo "Defaulting DATA_DIR=$DATA_DIR"
    echo "----------------------------------------------------------------------------"
else
    echo "----------------------------------------------------------------------------"
    echo "DATA_DIR has been defined as $DATA_DIR"
    echo "----------------------------------------------------------------------------"
fi

# --------------------- EXTRACTION OUTPUT BASE DIRECTORY --------------------- #
if [ -z ${OUTPUT_BASE_DIR+x} ]; then
    echo "----------------------------------------------------------------------------"
    echo "                      OUTPUT_BASE_DIR not defined.                          "
    OUTPUT_BASE_DIR=$OUTPUT_BASE_DIR_DEFAULT
    echo "Defaulting OUTPUT_BASE_DIR=$OUTPUT_BASE_DIR"
    echo "----------------------------------------------------------------------------"
else
    echo "----------------------------------------------------------------------------"
    echo "OUTPUT_BASE_DIR has been defined as $OUTPUT_BASE_DIR"
    echo "----------------------------------------------------------------------------"
fi

# ---------------------------- VERBOSITY SETTINGS ---------------------------- #
if [ -z ${VERBOSE+x} ]; then
    echo "----------------------------------------------------------------------------"
    VERBOSE=1
    echo "               VERBOSE not defined. Setting VERBOSE to $VERBOSE.                   "
    echo "----------------------------------------------------------------------------"
else
    echo "----------------------------------------------------------------------------"
    echo "                       VERBOSE has been defined as $VERBOSE                 "
    echo "----------------------------------------------------------------------------"
fi

# --------------------------- UNDISTORTION SETTINGS -------------------------- #
if [ -z ${UNDISTORT:+x} ]; then
    echo "----------------------------------------------------------------------------"
    UNDISTORT=0
    echo "              UNDISTORT not defined. Setting UNDISTORT to $UNDISTORT.                "
    echo "----------------------------------------------------------------------------"
else
    echo "----------------------------------------------------------------------------"
    echo "                      UNDISTORT has been defined as $UNDISTORT              "
    echo "----------------------------------------------------------------------------"
fi
if [ $UNDISTORT -eq 1 ]; then
    if [ -z ${CALIB_DIR+x} ]; then
        echo "----------------------------------------------------------------------------"
        echo "                         CALIB_DIR not defined.                             "
        CALIB_DIR="/home/roar/ART/perception/Camera/Calibration_new/"
        echo "Defaulting CALIB_DIR=$CALIB_DIR"
        echo "----------------------------------------------------------------------------"
    else
        echo "----------------------------------------------------------------------------"
        echo "CALIB_DIR has been defined as $CALIB_DIR"
        echo "----------------------------------------------------------------------------"
    fi
fi
# --------------------- ENVIRONMENT VARIABLE PARSING END --------------------- #

echo "\nPausing for 10 seconds. Press Ctrl+C to quit if the above settings are not correct.\n"
for i in 1 2 3 4 5 6 7 8 9 10
do
  echo "$i seconds passed"
  sleep 1s
done


# ---------------------------------------------------------------------------- #
#                 Find all rosbags at datadir and extract data                 #
# ---------------------------------------------------------------------------- #
echo "\n\nSearching DATA_DIR for ROSBAGS now...\n"
sleep 5s
find "$DATA_DIR" -iname "*.db3" -print0 | xargs -0 -I file dirname file | sort | uniq | while read d; do
    ROSBAG_NAME=$(basename "$d")

    # ---------------------------------------------------------------------------- #
    #                           NOTE: SPECIFY OUTPUT DIR                           #
    # ---------------------------------------------------------------------------- #
    OUTPUT_DIR="$OUTPUT_BASE_DIR$ROSBAG_NAME"
    
    # ------------------- NO CHANGES REQUIRED BEYOND THIS LINE ------------------- #

    # ------------------------------ Debug Verbosity ----------------------------- #
    echo "----------------------------------------------------------------------------"
    echo "Found ROSBAG:              $d"
    echo "Extracting images to:      $OUTPUT_DIR"
    echo "----------------------------------------------------------------------------"

    # -------------------------- Create output Directory ------------------------- #
    # mkdir -p $OUTPUT_DIR
    if [ $VERBOSE -eq 1 ]; then
        if [ $UNDISTORT -eq 1 ]; then
            python3 tools/rosbag_perception_dataset_maker/ros2bag_image_extractor.py "$d" $OUTPUT_DIR -vup $CALIB_DIR
        else 
            python3 tools/rosbag_perception_dataset_maker/ros2bag_image_extractor.py "$d" $OUTPUT_DIR -v
        fi
    else
        if [ $UNDISTORT -eq 1 ]; then
            python3 tools/rosbag_perception_dataset_maker/ros2bag_image_extractor.py "$d" $OUTPUT_DIR -up $CALIB_DIR
        else
            python3 tools/rosbag_perception_dataset_maker/ros2bag_image_extractor.py "$d" $OUTPUT_DIR
        fi
    fi
done
