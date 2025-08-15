import os
import shutil
import random

#TODO: Change the image names and take them in one folder

def rename_files(folder, prefix):
    for i, filename in enumerate(sorted(os.listdir(folder))):
        if filename.lower().endswith(('.jpg', '.jpg')):
            new_name = f"{prefix}_{i:05d}.jpg"
            os.rename(os.path.join(folder, filename), os.path.join(folder, new_name))

# This part just renames the images in the folder (in the ~/rgb) so when we merge there is not gonna be any problem with names
rename_files("synthtc_data/Replicator/rgb", "jetbot")
rename_files("synthtc_data/Replicator_01/rgb", "world")

# I will merge them into all_data/images folder
def merge_folders(source_folders, destination_folder):
    """
    Merges multiple folders into a single destination folder.
    
    Args:
        source_folders (list): List of source folder paths to merge.
        destination_folder (str): Path to the destination folder.
    """
    os.makedirs(destination_folder, exist_ok=True)
    
    for source_folder in source_folders:
        for filename in os.listdir(source_folder):
            if filename.lower().endswith(('.jpg', '.jpg')):
                src_file = os.path.join(source_folder, filename)
                dst_file = os.path.join(destination_folder, filename)
                shutil.copy(src_file, dst_file)

# Merge the folders
source_folders = ["synthtc_data/Replicator/rgb", "synthtc_data/Replicator_01/rgb"]
destination_folder = "all_data/images"
merge_folders(source_folders, destination_folder)

#TODO: Change the instance id segmentation labels from json to txt format for YOLO

import json
import cv2
import numpy as np

# Map object keywords to YOLO class IDs
CLASS_MAP = {
    "forklift": 0,
    "pallet": 1,
    "conveyor_belt": 2,
    "jetbot": 3
}

def parse_color_key(key):
    return tuple(map(int, key.strip("()").split(",")))

def color_to_class(path):
    for k, v in CLASS_MAP.items():
        if k in path:
            return v
    return None

def json_mask_to_yolo(json_path, mask_path, output_path):
    # Load JSON mapping
    with open(json_path) as f:
        color_map = {parse_color_key(k): v for k, v in json.load(f).items()}

    # Load mask image (RGBA)
    mask = cv2.imread(mask_path, cv2.IMREAD_UNCHANGED)
    h, w = mask.shape[:2]

    lines = []
    for color, obj_path in color_map.items():
        if obj_path == "INVALID":
            continue
        cls_id = color_to_class(obj_path)
        if cls_id is None:
            continue

        # Binary mask for this color
        lower = np.array(color, dtype=np.uint8)
        upper = np.array(color, dtype=np.uint8)
        obj_mask = cv2.inRange(mask, lower, upper)

        # Find contours
        contours, _ = cv2.findContours(obj_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if len(cnt) < 3:
                continue
            polygon = cnt.reshape(-1, 2)
            # Normalize
            norm_poly = []
            for x, y in polygon:
                norm_poly.append(f"{x/w:.6f}")
                norm_poly.append(f"{y/h:.6f}")
            lines.append(f"{cls_id} " + " ".join(norm_poly))

    # Save TXT
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        f.write("\n".join(lines))

# ---- Batch process all files ----
json_folder = "synthtc_data/Replicator_01/instance_id_segmentation"
mask_folder = "synthtc_data/Replicator_01/instance_id_segmentation"   # same folder in your case
output_labels_folder = "Label/world"

for file in os.listdir(json_folder):
    if file.endswith(".json") and file.startswith("instance_id_segmentation_mapping_"):
        # Get just the frame number, e.g. "0268"
        frame_id = file.replace("instance_id_segmentation_mapping_", "").replace(".json", "")
        
        # Match jpg name
        mask_name = f"instance_id_segmentation_{frame_id}.jpg"
        mask_path = os.path.join(mask_folder, mask_name)
        
        # Set YOLO label name (still using frame_id for now)
        yolo_label_name = f"{frame_id}.txt"
        output_path = os.path.join(output_labels_folder, yolo_label_name)

        if os.path.exists(mask_path):
            json_mask_to_yolo(
                os.path.join(json_folder, file),
                mask_path,
                output_path
            )
            print(f"Converted {file} → {yolo_label_name}")
        else:
            print(f"Mask not found for {file}")


# Merge the labels and rename them to match image filenames
def merge_labels(source_folders, destination_folder, prefixes):
    """
    Merges and renames label files from multiple folders into a single destination folder.

    Args:
        source_folders (list): List of source folder paths containing label files.
        destination_folder (str): Path to the destination folder for merged labels.
        prefixes (list): List of prefixes to add (e.g., 'jetbot', 'world'),
                         must match order of source_folders.
    """
    os.makedirs(destination_folder, exist_ok=True)
    
    for source_folder, prefix in zip(source_folders, prefixes):
        for filename in os.listdir(source_folder):
            if filename.lower().endswith('.txt'):
                # Extract number part and pad with zeros if needed
                number = filename.replace('.txt', '')
                number = number.zfill(5)  # ensures 00001 format
                
                # New filename to match image name format
                new_filename = f"{prefix}_{number}.txt"
                
                src_file = os.path.join(source_folder, filename)
                dst_file = os.path.join(destination_folder, new_filename)
                
                shutil.copy(src_file, dst_file)
                print(f"Copied {src_file} → {dst_file}")

# Example usage
source_folders = ["Label/jetbot", "Label/world"]
prefixes = ["jetbot", "world"]
destination_folder = "all_data/labels"

merge_labels(source_folders, destination_folder, prefixes)

#! Just to let you know there is definitely better ways to do this, but I just take those images and txt to the main folder and I will split that way


#TODO: Split The Data

def split_dataset(input_folder, output_folder, train_ratio=0.7, val_ratio=0.2, test_ratio=0.1):
    # Create output directories
    for split in ['train', 'val', 'test']:
        os.makedirs(os.path.join(output_folder, split, 'images'), exist_ok=True)
        os.makedirs(os.path.join(output_folder, split, 'labels'), exist_ok=True)

    # Get all image files
    frame_files = [f for f in os.listdir(input_folder) if f.endswith('.jpg')]
    random.shuffle(frame_files)

    total_files = len(frame_files)
    train_split = int(total_files * train_ratio)
    val_split = train_split + int(total_files * val_ratio)

    splits = {
        'train': frame_files[:train_split],
        'val': frame_files[train_split:val_split],
        'test': frame_files[val_split:]
    }

    for split, files in splits.items():
        for file_name in files:
            # Copy image
            shutil.copy(
                os.path.join(input_folder, file_name),
                os.path.join(output_folder, split, 'images', file_name)
            )

            # Copy label (if it exists)
            label_file = file_name.replace('.jpg', '.txt')
            label_path = os.path.join(input_folder, label_file)
            if os.path.exists(label_path):
                shutil.copy(
                    label_path,
                    os.path.join(output_folder, split, 'labels', label_file)
                )

    print(f"✅ Dataset split completed: {output_folder}")

# Example usage:
split_dataset(
    input_folder='all_data',
    output_folder='final_data'
)

#! DON'T FORGOT TO CREATE data.yaml FILE

#! We will train our custom YOLO model from terminal with the following command:
#TODO: yolo task=segment mode=train model=yolov8s-seg.pt data=data.yaml epochs=100 imgsz=640

# After training we can use best.pt model in the weights folder

