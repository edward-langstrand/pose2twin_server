import argparse
import subprocess
import os
import cv2
import shutil
from pathlib import Path


def split_video_grid(video_path, out_dir, video_columns, video_rows, camera_count):
    out_dir.mkdir(exist_ok=True, parents=True)
    
    capture = cv2.VideoCapture(str(video_path), apiPreference=cv2.CAP_FFMPEG)
    frame_idx = 0
    while (True):
        success, frame = capture.read()
        
        if success:
            # Split frame containing all camera views into separate frames and store them in directories corresponding to camera id
            total_pixel_rows = frame.shape[0]
            total_pixel_columns = frame.shape[1]
            cam_pixel_rows = total_pixel_rows//video_rows
            cam_pixel_columns = total_pixel_columns//video_columns
            
            for cam_index in range(camera_count):
                row = cam_index // video_rows
                column = cam_index % video_columns
                
                image_path = Path(f'{out_dir}/{cam_index}/{frame_idx}.png')
                image_path.parent.mkdir(exist_ok=True, parents=True)
                
                camera_frame = frame[row*cam_pixel_rows:(row+1)*cam_pixel_rows, column*cam_pixel_columns:(column+1)*cam_pixel_columns:, :]
                cv2.imwrite(filename=str(image_path), img=camera_frame)
        else:
            break
    
        frame_idx = frame_idx+1
    
    capture.release()


def undistort_images(image_dir, out_dir, camera_parameter_dir, cam_id):
    subprocess.call([openpose_bin, '--frame_undistort', '--image_dir', image_dir/str(cam_id), '--write_images', out_dir, '--camera_parameter_path', camera_parameter_dir/f'{cam_id}.xml', '--num_gpu', '0', '--display', '0'], cwd=openpose_dir)


def calibrate_extrinsics(camera_parameter_dir, calibration_image_dir, cam0, cam1, grid_square_size_mm, grid_number_inner_corners, combine_cam0_extrinsics):
    paramters = [calibration_bin, 
                 '--mode', '2', 
                 '--grid_square_size_mm', str(grid_square_size_mm), 
                 '--grid_number_inner_corners', grid_number_inner_corners, 
                 '--calibration_image_dir', calibration_image_dir, 
                 '--camera_parameter_folder', str(camera_parameter_dir)+'/', 
                 '--cam0', str(cam0),
                 '--cam1', str(cam1),
                 '--omit_distortion',
                 '--logging_level', '2']
    
    print(paramters)
    
    if combine_cam0_extrinsics:
        paramters.append('--combine_cam0_extrinsics')
    
    subprocess.call(paramters, cwd=openpose_dir)


def refine_calibration(camera_parameter_dir, calibration_image_dir, cam_count, grid_square_size_mm, grid_number_inner_corners):
    subprocess.call([calibration_bin, 
                 '--mode', '3', 
                 '--grid_square_size_mm', str(grid_square_size_mm), 
                 '--grid_number_inner_corners', grid_number_inner_corners, 
                 '--calibration_image_dir', calibration_image_dir, 
                 '--camera_parameter_folder', str(camera_parameter_dir)+'/', 
                 '--number_cameras', str(cam_count),
                 '--omit_distortion'], cwd=openpose_dir)
    
    
def camera_pair(cam0, cam1, camera_pairs):
    result = False
    
    for pair_idx in range(len(camera_pairs)//2):
        pair0 = camera_pairs[pair_idx*2]
        pair1 = camera_pairs[pair_idx*2+1]
        
        if pair0 == cam0 and pair1 == cam1:
            result == True
        elif pair1 == cam0 and pair0 == cam1:
            result = True
    
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Uses OpenPose to calibrate camera intrinsics')
    parser.add_argument('-c','--cam_origin', help='The identifier for the camera that will be the origin', required=True, type=int)
    parser.add_argument('-v','--video', help='Path to video which will be used for calibration', required=True)
    parser.add_argument('-vc','--video_columns', help='The amount of cameras in the horizontal axis', required=False, default=2, type=int)
    parser.add_argument('-vr','--video_rows', help='The amount of cameras in the vertical axis', required=False, default=2, type=int)
    parser.add_argument('-cc','--cam_count', help='The amount of cameras', required=False, default=4, type=int)
    parser.add_argument('-gss','--grid_square_size_mm', help='Size per square in mm on the checker board', required=False, default=41, type=int)
    parser.add_argument('-gnic','--grid_number_inner_corners', help='Grid size of the checker board\'s inner corners', required=False, default='8x6')
    parser.add_argument('-fcp','--force_camera_pairs', help='Provide a list of pairs of cameras that are to be paired. The first one is the camera that will be calibrated relative to the second one. E.g. (-fcp 3 4 1 2)', required=False, nargs='*', type=int, default=[])
    args = vars(parser.parse_args())

    # Check that force camera pairs are provided in pairs
    force_camera_pairs = args['force_camera_pairs']
    if len(force_camera_pairs) % 2 != 0:
        print('Force camera pairs must be provided as pairs of two camera ids')
        exit(1)
        
    # Check that origin camera is not forced
    if args['cam_origin'] in force_camera_pairs:
        print('Origin camera can not be forced, it will be paired to origin if it can anyway')
        exit(1)
        
    # Construct two-way map of forced camera pairs for easier lookup later
    force_map = dict()
    for force_cam0 in range(len(force_camera_pairs)//2):
        force_map.update({force_camera_pairs[force_cam0]: force_camera_pairs[force_cam0+1]})
        # force_map.update({force_camera_pairs[force_cam0+1]: force_camera_pairs[force_cam0]})

    script_dir = Path(__file__).parent.resolve()
    openpose_dir = (script_dir/'../openpose').resolve()
    openpose_bin = (openpose_dir/'build/examples/openpose/openpose.bin')
    calibration_bin = (openpose_dir/'build/examples/calibration/calibration.bin')
    intrinsic_calibration_dir = (script_dir/'calibration/intrinsic')
    final_calibration_dir = (script_dir/'calibration/final')
    intermediate_dir = (script_dir/'intermediate')
    calibration_image_dir = (intermediate_dir/'extrinsic')
    combined_calibration_image_dir = (calibration_image_dir/'combined')

    # Cleanup the images in intermediate to prevent overlapping video frames
    if calibration_image_dir.exists():
        shutil.rmtree(calibration_image_dir)
        
    # Split the combined video into individual images
    split_video_grid(Path(args['video']).resolve(), calibration_image_dir, args['video_columns'], args['video_rows'], args['cam_count'])
    
    for cam_idx in range(args['cam_count']):
        # Undistort images for camera based on intrinsics calibration
        undistored_path = calibration_image_dir/str(cam_idx)/'undistored'
        undistort_images(calibration_image_dir, undistored_path, intrinsic_calibration_dir, cam_idx)
        
        # Move undistored images together and rename to the OpenPose extrinsic calibration convention
        for undistored_image in undistored_path.iterdir():
            if cam_idx != 0:
                new_path = combined_calibration_image_dir/f'{undistored_image.stem}_{cam_idx}.{undistored_image.suffix}'
            else:
                new_path = combined_calibration_image_dir/undistored_image.name
                
            new_path.parent.mkdir(exist_ok=True, parents=True)
            undistored_image.rename(new_path)
    
    # Cleanup paramter xml's so that amount of parameters matches current cameras
    if final_calibration_dir.exists():
        shutil.rmtree(final_calibration_dir)
    final_calibration_dir.mkdir(exist_ok=True, parents=True)
    
    # Copy intrinsic calibration files to not disturb them and get a clean slate for extrinsics
    for cam_id in range(args['cam_count']):
        shutil.copy(intrinsic_calibration_dir/f'{cam_id}.xml', final_calibration_dir/f'{cam_id}.xml')
        
    # Resolve extrinsic calibration by avoiding opposite camera pairs and starting with camera origin
    calibrated_cams = set()
    
    # Start by pairing all cameras to origin camera if possible
    for cam1 in range(args['cam_count']):
        if cam1==args['cam_origin']: continue
        
        if cam1 not in force_map:
            calibrate_extrinsics(final_calibration_dir, combined_calibration_image_dir, args['cam_origin'], cam1, args['grid_square_size_mm'], args['grid_number_inner_corners'], False)
            # print(str(args['cam_origin']) + " - " + str(cam1))
            calibrated_cams.add(args['cam_origin'])
            calibrated_cams.add(cam1)
        
    # If any cameras remain, pair them with an already calibrated camera
    while len(calibrated_cams) < args['cam_count']:
        
        for cam1 in range(args['cam_count']):
            # Find an uncalibrated camera as cam1
            if cam1 not in calibrated_cams:
        
                # Get the assigned reference camera for cam1
                if cam1 in force_map:
                    cam0 = force_map.get(cam1)
                else:
                    print(f'Failed to calibrate extrinsics for camera {cam1}, make sure to force pair it with a camera that is already calibrated')
                
                # If the camera to use as reference is not calibrated yet, move on to next and come back to this one later
                if cam0 in calibrated_cams:
                    calibrate_extrinsics(final_calibration_dir, combined_calibration_image_dir, cam0, cam1, args['grid_square_size_mm'], args['grid_number_inner_corners'], True)
                    calibrated_cams.add(cam1)

        
    # refine_calibration(final_calibration_dir, combined_calibration_image_dir, args['cam_count'], args['grid_square_size_mm'], args['grid_number_inner_corners'])
