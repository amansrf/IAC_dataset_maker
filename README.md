# IAC_dataset_maker
A simple repository with a pipeline to identify and extract important camera data to be labelled.

## Dataset Pipeline
Use the following steps to extract data from a rosbag, remove image distortions, choose image segments with vehicles, and create a dataset to be labelled:
1. [multi_extract.sh](multi_extract.sh): This is a bash script to extract images from a rosbag and optionally undistort them during extraction. To set his up for your own rosbag follow the following steps:
    1. Specify the extraction environment variables:
        * DATA_DIR_DEFAULT: Directory where rosbags exist. We will search this directory for db3 files to determine where these rosbags are. 
        * VERBOSE_DEFAULT: Set to 1 to get verbose extraction.
        * UNDISTORT_DEFAULT: Set to 1 to get undistortion during the extraction process.
        * CALIB_DIR_DEFAULT: If you choose to undistort, pass the path to the directory with the calibration files.
    2. `[OPTIONAL]` Instead of specifying the DEFAULT VARIABLES as in step 1, you may export environment variables in the bash shell before running. Here is an example:
        * ```bash
            export DATA_DIR=<PATH_TO_ROSBAGS_DIR>
            export VERBOSE=1
            export UNDISTORT=1
            export CALIB_DIR_DEFAULT=<PATH_TO_CALIBRATION_FILES>
            ```
    3. Make sure the topic names you want to extract exist in the [ros2bag_image_extractor.py](ros2bag_image_extractor.py) in the iterator dictionary.
    4. Run multi_extract.sh:
        ```bash
            bash multi_extract.sh
        ```
    5. Now sit back and wait until the extraction is complete. This process can take and hour or longer depending on the size of the rosbag.

2. Convert the extracted image data to a video to view the segments that contain the car. This is important to reduce the dataset size that is sent to an external vendor for labelling, saving cost and time. Use the following command in the OUTPUT_DIR printed by the previous command:
    ```bash
    ffmpeg -framerate 50 -pattern_type glob -i '*.jpg' -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p <NAME_AND_PATH_OF_VIDEO_FILE_OUTPUT.mp4>
    ```
3. `[COMPLETELY MANUAL]` Review the video files and note down the timestamps where the vehicle starts being visible and where it stops being visible. Convert the timestamp into seconds format and multiply by 50 (The frame rate we set in the previous step) to get the sequence number for each start and stop position. 
    - `For example a time of 1:10 (1 minute and 10 seconds) in the video viewer translates to 70 seconds and a frame ID of 3500 (70*frame_rate, where frame_rate is 50 for our example)`
4. [dataset_maker.py](dataset_maker.py): Next we we will extract these important segments into a seperate folder for final extraction and dataset creation. 
    - Input the sequence numbers from the previous step into the [dataset_maker.py](dataset_maker.py) file in the `ranges` array in pairs, for example: 
        ```python
        ranges = np.array(
            [
                0, 3500,
                30000, 43500,
                66000, 74250,
                86500, 87750,
            ]
        )
        ```
    - Set the source directory and destination directory in the [dataset_maker.py](dataset_maker.py) file:
        ```python
            SOURCE_DIR = "<ONE OF THE EXTRACTED DATA DIRS FROM STEP 1>"
            DEST_DIR = "<PATH WHERE DATASET IS TO BE OUTPUT>"
        ```
    - Now run the python file 
        ```
            python3 dataset_maker.py
        ```
    
# Contact
* Aman Saraf | [amansaraf99@gmail.com](mailto:amansaraf99@gmail.com)

    


