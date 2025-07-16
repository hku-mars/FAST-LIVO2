import os
import mediapy as media
import argparse
from tqdm import tqdm
import cv2
import numpy as np


def create_video_from_images(base_path):
    if os.path.exists(base_path):
        image0_dir = os.path.join(base_path, 'images', 'image_00')
        image1_dir = os.path.join(base_path, 'images', 'image_01')
        image2_dir = os.path.join(base_path, 'images', 'image_02')
        image3_dir = os.path.join(base_path, 'images', 'image_03')
        
        image0_files = sorted(os.listdir(image0_dir))
        image1_files = sorted(os.listdir(image1_dir))
        image2_files = sorted(os.listdir(image2_dir))
        image3_files = sorted(os.listdir(image3_dir))
        
        height, width, _ = cv2.imread(os.path.join(image0_dir, image0_files[0])).shape
        filename = 'multiview_video.mp4'
        
        idx = 0
        with media.VideoWriter(
            path=os.path.join(base_path, filename), fps=10, shape=(height*2, width*3)
        ) as writer:
            for image0, image1, image2, image3 in tqdm(zip(image0_files, image1_files, image2_files, image3_files), desc="Writing videos..", total=len(image0_files), leave=False):
                img0 = cv2.imread(os.path.join(image0_dir, image0))
                img0 = cv2.cvtColor(img0, cv2.COLOR_BGR2RGB)
                
                img1 = cv2.imread(os.path.join(image1_dir, image1))
                img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
                
                img2 = cv2.imread(os.path.join(image2_dir, image2))
                img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
                img2 = cv2.flip(img2, 1) # flip the image on backside
                
                img3 = cv2.imread(os.path.join(image3_dir, image3))
                img3 = cv2.cvtColor(img3, cv2.COLOR_BGR2RGB)
                
                blank = np.zeros((int(height/2), width, 3), np.uint8)
                
                # Concatenate image_rgb and pose_rgb horizontally
                combined_image1 = cv2.vconcat([blank, img1, blank])
                combined_image2 = cv2.vconcat([img0, img2])
                combined_image3 = cv2.vconcat([blank, img3, blank])
                final_combined_image = cv2.hconcat([combined_image1, combined_image2, combined_image3])
                
                # Add the combined image to the video writer
                writer.add_image(final_combined_image)
                idx += 1


if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Create a video from images in specified directories.")
    parser.add_argument("--input_path", "-i", type=str, help="The base path for the image directories.")
    
    # Parse the arguments
    args = parser.parse_args()
    input_path = args.input_path
    
    create_video_from_images(input_path)
