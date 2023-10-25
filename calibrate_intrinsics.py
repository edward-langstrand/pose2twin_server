import argparse
import subprocess
import os
import cv2
import shutil
from pathlib import Path


def split_video(video_path, out_dir):
    out_dir.mkdir(exist_ok=True, parents=True)
    
    capture = cv2.VideoCapture(str(video_path), apiPreference=cv2.CAP_FFMPEG)
    frame_idx = 0
    while (True):
        success, frame = capture.read()
        
        if success:
            cv2.imwrite(filename=f'{out_dir}/{frame_idx}.png', img=frame)
        else:
            break
    
        frame_idx = frame_idx+1
    capture.release()


def calibrate_intrinsics(camera_parameter_dir, calibration_image_dir, camera_id, grid_square_size_mm, grid_number_inner_corners):
    subprocess.call([calibration_bin, 
                 '--mode', '1', 
                 '--grid_square_size_mm', str(grid_square_size_mm), 
                 '--grid_number_inner_corners', grid_number_inner_corners, 
                 '--calibration_image_dir', calibration_image_dir, 
                 '--camera_parameter_folder', camera_parameter_dir, 
                 '--camera_serial_number', str(camera_id)], cwd=openpose_dir)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Uses OpenPose to calibrate camera intrinsics')
    parser.add_argument('-c','--cam_id', help='The identifier for the camera', required=True, type=int)
    parser.add_argument('-v','--video', help='Path to video which will be used for calibration', required=True)
    parser.add_argument('-gss','--grid_square_size_mm', help='Size per square in mm on the checker board', required=False, default=41, type=int)
    parser.add_argument('-gnic','--grid_number_inner_corners', help='Grid size of the checker board\'s inner corners', required=False, default='8x6')
    args = vars(parser.parse_args())


    script_dir = Path(__file__).parent.resolve()
    openpose_dir = (script_dir/'../openpose').resolve()
    openpose_bin = (openpose_dir/'build/examples/openpose/openpose.bin')
    calibration_bin = (openpose_dir/'build/examples/calibration/calibration.bin')
    intrinsic_calibration_dir = (script_dir/'calibration/intrinsic')
    intermediate_dir = (script_dir/'intermediate')
    calibration_image_dir = (intermediate_dir/str(args['cam_id']))

    # Cleanup the images in intermediate to prevent overlapping video frames
    if calibration_image_dir.exists():
        shutil.rmtree(calibration_image_dir)
    
    split_video(Path(args['video']).resolve(), calibration_image_dir)
    calibrate_intrinsics(intrinsic_calibration_dir, calibration_image_dir, args['cam_id'], args['grid_square_size_mm'], args['grid_number_inner_corners'])
