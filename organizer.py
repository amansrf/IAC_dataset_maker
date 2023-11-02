import os
import shutil

# Define the paths
first_path = '/home/art-berk/IAC_dataset_maker/output'
second_path = '/media/art-berk/DRIVE2_ART/rosbags'

# Ensure 'rosbag_with_images_extracted' directory exists
destination_folder = os.path.join(second_path, 'rosbag_with_images_extracted')
os.makedirs(destination_folder, exist_ok=True)

# Get a set of all directory names in first path
first_path_dirs = {dir_name for dir_name in os.listdir(first_path)
                   if os.path.isdir(os.path.join(first_path, dir_name))}

# Iterate over all directories in the second path
for dir_name in os.listdir(second_path):
    dir_path = os.path.join(second_path, dir_name)
    # Check if it's a directory and not the destination directory
    if dir_name not in first_path_dirs or not os.path.isdir(dir_path) or dir_path == destination_folder:
        continue  # Skip if no match or it's not a directory or if it is the destination directory
    
    # Move matched directory to the 'rosbag_with_images_extracted' directory
    shutil.move(dir_path, destination_folder)
    print(f"Moved '{dir_name}' to '{destination_folder}'")

print("Operation completed.")
