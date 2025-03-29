import os
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

def convert_slam_to_prism(slam_tum_file, output_file):
    """
    Convert SLAM trajectory in TUM format to prism coordinate system
    :param slam_tum_file: SLAM estimated trajectory file (TUM format, quaternion order x y z w)
    :param output_file: Output file path
    """
    try:
        # Check file existence
        if not os.path.exists(slam_tum_file):
            raise FileNotFoundError(f"SLAM file not found: {slam_tum_file}")

        # Read SLAM trajectory data
        slam_data = pd.read_csv(slam_tum_file, sep=' ', header=None)
        timestamps = slam_data.iloc[:, 0].values
        positions = slam_data.iloc[:, 1:4].values
        quaternions = slam_data.iloc[:, 4:8].values  # Quaternion order: x y z w

        # Static transformation vector from body to prism coordinates
        trans_B2prism = np.array([-0.293656, -0.012288, -0.273095])

        # Convert positions to prism coordinate system
        positions_prism = positions + quat_rotate_vector(quaternions, trans_B2prism)

        # Save in TUM format
        output_data = np.column_stack((timestamps, positions_prism, quaternions))
        np.savetxt(output_file, output_data, fmt='%.6f', delimiter=' ')
        print(f"Successfully processed SLAM data: {output_file}")

    except Exception as e:
        print(f"Error processing SLAM data: {str(e)}")
        raise

def convert_leica_to_tum(leica_file, output_file):
    """
    Convert leica_pose.csv to TUM format
    :param leica_file: leica_pose.csv file path
    :param output_file: Output file path
    """
    try:
        # Check file existence
        if not os.path.exists(leica_file):
            raise FileNotFoundError(f"Leica file not found: {leica_file}")

        # Read Leica data
        leica_data = pd.read_csv(leica_file, skiprows=1, header=None).values
        timestamps = leica_data[:, 0] / 1e9  # Convert nanoseconds to seconds
        positions = leica_data[:, 3:6]
        quaternions = np.zeros((len(positions), 4))  # Assume identity rotation
        quaternions[:, 3] = 1  # w-component of quaternion

        # Save in TUM format
        output_data = np.column_stack((timestamps, positions, quaternions))
        np.savetxt(output_file, output_data, fmt='%.6f', delimiter=' ')
        print(f"Successfully processed Leica data: {output_file}")

    except Exception as e:
        print(f"Error processing Leica data: {str(e)}")
        raise

def quat_rotate_vector(quat, vec):
    """
    Rotate vector using quaternion(s) (quaternion order: x y z w)
    :param quat: Quaternion array (N x 4, order [x, y, z, w])
    :param vec: 3D vector to rotate
    :return: Rotated vectors (N x 3)
    """
    # Create rotation object using x y z w order
    rot = Rotation.from_quat(quat[:, :4])
    return rot.apply(vec)

if __name__ == '__main__':
    # File paths
    input_files = {
        'slam': 'eee_01.txt',
        'leica': 'leica_pose.csv'
    }
    
    output_files = {
        'slam': 'eee_01_prism.txt',
        'leica': 'eee_01_gt.txt'
    }

    # Process SLAM data
    try:
        convert_slam_to_prism(input_files['slam'], output_files['slam'])
    except:
        print("Skipping SLAM processing")

    # Process Leica data
    try:
        convert_leica_to_tum(input_files['leica'], output_files['leica'])
    except:
        print("Skipping Leica processing")
