import os 
import shutil
import numpy as np
from tqdm import tqdm


ranges = np.array(
    [
        0, 150,
        30000, 43500,
        66000, 74250,
        86500, 87750,
    ]
)

SOURCE_DIR = "/media/Public/cam_tms/rosbag2_2022_09_30-16_39_02/vimba_rear_left"
DEST_DIR = "/media/Public/cam_tms/rosbag2_2022_09_30-16_39_02/TO_BE_LABELLED/vimba_rear_left"

print("Scanning Source Directory: " + SOURCE_DIR + "\n")
print("This operation may take a while if source directory has too many files...\n\n")
source_file_list = os.listdir(SOURCE_DIR)
print("Scan Done")

count = 0
total_to_be_copied = 0
for i in range(0,ranges.shape[0],2):
    total_to_be_copied += ranges[i+1] - ranges[i]

print(f"Max files to be copied are {total_to_be_copied} out of a total of {len(source_file_list)}")
print(f"This means you should expect the total time to be {100*total_to_be_copied//len(source_file_list)}% of the time displayed in the beginning")

print("Now copying data from " + SOURCE_DIR + "to " + DEST_DIR + "\n")

for tqdm_iter in tqdm(range(len(source_file_list))):
    source_file = source_file_list[tqdm_iter]
    file_words = source_file.split('_')
    for i in range(0,ranges.shape[0],2):
        begin, end = ranges[i], ranges[i+1]

        if int(file_words[1]) in range(begin, end):
            # print(file_words, int(file_words[1]), source_file, begin, end)
            shutil.copy(SOURCE_DIR+'/'+source_file, DEST_DIR)
            count += 1
            break

f= open(DEST_DIR+"/num_data.txt","w+")
f.write(f"{count}")
f.close()