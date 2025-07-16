import cv2
from tqdm import tqdm
import numpy as np
import argparse
import json
import shutil
import os

def rectify_fisheye_image(image_path, K, D):
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError("Image not found!")

    # Get the dimensions of the image
    h, w = image.shape[:2]

    # Compute the new camera matrix for undistortion
    zoom_out = 1.0
    K_new = K.copy()
    K_new[0, 0] *= zoom_out
    K_new[0, 1] = 0.0
    K_new[1, 1] *= zoom_out
    
    # K_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=0.0)

    # Create a mapping for undistortion
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K_new, (w, h), cv2.CV_16SC2)

    # Rectify the image
    rectified_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    return K_new, rectified_image

def process_rectification(args):
    root_path = args.root_path
    input_path = args.input_path
    
    CAMERA_LIST = [0, 1, 2, 3]

    for cam_id in tqdm(CAMERA_LIST):
        intrinsic_file = os.path.join(root_path, 'calibration', 'cam_%02d.json' % cam_id)
        extrinsic_file = os.path.join(root_path, 'calibration', 'cams_to_lidar.txt')
        
        with open(intrinsic_file, 'r') as file:
            intrinsic_data = json.load(file)

        # Camera intrinsic matrix
        K = np.array(intrinsic_data["K"])

        # Distortion coefficients
        D = np.array(intrinsic_data["D"][0])
        
        image_dir = os.path.join(input_path, 'images', 'image_%02d' % cam_id)
        image_out_dir = os.path.join(input_path, 'rectified', 'image_%02d' % cam_id)
        os.makedirs(image_out_dir, exist_ok=True)
        
        image_files = sorted([f for f in os.listdir(image_dir) if f.endswith('.jpg')])
        
        for idx, image_file in enumerate(tqdm(image_files, leave=True, desc=f"Rectifying images from camera {cam_id}")):
            image_path = os.path.join(image_dir, image_file)
            K_new, rectified_image = rectify_fisheye_image(image_path, K, D)
            
            if idx == 0:
                os.makedirs(os.path.join(input_path, 'params'), exist_ok=True)
                os.makedirs(os.path.join(input_path, 'rectified_params'), exist_ok=True)
                shutil.copy2(intrinsic_file, os.path.join(input_path, 'params', 'cam_%02d.json' % cam_id))
                np.savetxt(os.path.join(input_path, 'rectified_params', 'cam_%02d.txt' % cam_id), K_new, fmt='%.6f')
            
            output_path = os.path.join(image_out_dir, image_file)
            cv2.imwrite(output_path, rectified_image)

    shutil.copy2(extrinsic_file, os.path.join(input_path, 'params', 'cams_to_lidar.txt'))
    shutil.copy2(extrinsic_file, os.path.join(input_path, 'rectified_params', 'cams_to_lidar.txt'))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "Rectify fisheye images",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--root_path", "-r", type=str, required=True, help="Root directory of the dataset")
    parser.add_argument("--input_path", "-i", type=str, required=True, help="Path to the input data")
    args = parser.parse_args()
    
    process_rectification(args)