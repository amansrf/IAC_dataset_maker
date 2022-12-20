#!/bin/sh

# ---------------------------------------------------------------------------- #
#                               DEFAULT VARIABLES                              #
# ---------------------------------------------------------------------------- #
DATA_DIR_DEFAULT="/media/Public/ROSBAG_BACKUPS/rosbag2_2022_09_21-12_58_49"
VERBOSE_DEFAULT=1
UNDISTORT_DEFAULT=1
CALIB_DIR_DEFAULT="/home/roar/ART/perception/Camera/Calibration_new/"
OUTPUT_BASE_DIR_DEFAULT="/media/Public/Lucas_Oil/"
MAKE_VID_DEFAULT=1

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
    VERBOSE=$VERBOSE_DEFAULT
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
    UNDISTORT=$UNDISTORT_DEFAULT
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
        CALIB_DIR=$CALIB_DIR_DEFAULT
        echo "Defaulting CALIB_DIR=$CALIB_DIR"
        echo "----------------------------------------------------------------------------"
    else
        echo "----------------------------------------------------------------------------"
        echo "CALIB_DIR has been defined as $CALIB_DIR"
        echo "----------------------------------------------------------------------------"
    fi
fi

# ---------------------------- MAKE_VID SETTINGS ---------------------------- #
if [ -z ${MAKE_VID+x} ]; then
    echo "----------------------------------------------------------------------------"
    MAKE_VID=$MAKE_VID_DEFAULT
    echo "               MAKE_VID not defined. Setting MAKE_VID to $MAKE_VID_DEFAULT.                   "
    echo "----------------------------------------------------------------------------"
else
    echo "----------------------------------------------------------------------------"
    echo "                       MAKE_VID has been defined as $MAKE_VID                 "
    echo "----------------------------------------------------------------------------"
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
    mkdir -p $OUTPUT_DIR

    # ------------------ Begin Extraction Based on User Setting ------------------ #
    if [ $VERBOSE -eq 1 ]; then
        if [ $UNDISTORT -eq 1 ]; then
            python3 ros2bag_image_extractor.py "$d" $OUTPUT_DIR -vup $CALIB_DIR
        else 
            python3 ros2bag_image_extractor.py "$d" $OUTPUT_DIR -v
        fi
    else
        if [ $UNDISTORT -eq 1 ]; then
            python3 ros2bag_image_extractor.py "$d" $OUTPUT_DIR -up $CALIB_DIR
        else
            python3 ros2bag_image_extractor.py "$d" $OUTPUT_DIR
        fi
    fi

    # --------------------- Convert Extracted Images to Video -------------------- #
    if [ $MAKE_VID -eq 1 ]; then
        for camera_output_dir in $OUTPUT_DIR/*/; do
            cd $camera_output_dir
            base_name=$(basename "$camera_output_dir")
            ffmpeg -framerate 50 -pattern_type glob -i '*.jpg' -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p ../$base_name.mp4
        done
    fi

done
