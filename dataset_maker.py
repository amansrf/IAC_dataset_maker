import os 
import shutil
import numpy as np
from tqdm import tqdm


ranges = np.array(
    [
        3500, 3750,
        6000, 7000,
        8000, 8500,
        9750, 11000,
        11250, 11750,
        13000, 13750,
        14000, 14750,
    ]
)

SOURCE_DIR = "/media/Public/cam_tms/rosbag2_2022_11_09-16_27_25/vimba_front_right_center"
DEST_DIR = "/media/Public/cam_tms/rosbag2_2022_11_09-16_27_25/TO_BE_LABELLED/vimba_front_right_center"

# ------------- Create Destination Directory if it does not exist ------------ #
if not os.path.isdir(DEST_DIR):
    print(f"DEST_DIR ({DEST_DIR}) did not exist so creating it.")
    os.makedirs(DEST_DIR)


print("Scanning Source Directory: " + SOURCE_DIR + "\n")
print("This operation may take a while if source directory has too many files...\n\n")
source_file_list = os.listdir(SOURCE_DIR)
print("Scan Done")

count = 0
total_to_be_copied = 0
iter_array = None
for i in range(0,ranges.shape[0],2):
    if iter_array is None:
        iter_array = np.arange(ranges[i], ranges[i+1], dtype=np.uint)
    else:
        iter_array = np.append(iter_array, np.arange(ranges[i], ranges[i+1], dtype=np.uint))
    total_to_be_copied += ranges[i+1] - ranges[i]

print("Now copying data from:\n" + SOURCE_DIR + "\nto:\n" + DEST_DIR + "\n")

print(f"Max files to be copied are {total_to_be_copied} out of a total of {len(source_file_list)}")

for tqdm_iter in tqdm(iter_array):
    source_file = source_file_list[tqdm_iter]
    file_words = source_file.split('_')

    if not os.path.exists(os.path.join(DEST_DIR, source_file)):
        shutil.copy(SOURCE_DIR+'/'+source_file, DEST_DIR)
    count += 1

print(count)
f= open(DEST_DIR+"/num_data.txt","w+")
f.write(f"{count}")
f.close()