import sys
import cv2
import numpy as np
import NDIlib as ndi
import argparse
from pose_to_twin_api import JointType, Pose2TwinServer
from pathlib import Path


def split_image(image, rows, cols, count):
    result = []
    
    total_pixel_rows = image.shape[0]
    total_pixel_columns = image.shape[1]
    cam_pixel_rows = total_pixel_rows//rows
    cam_pixel_columns = total_pixel_columns//cols
    
    for cam_index in range(count):
        row = cam_index // rows
        column = cam_index % cols
        
        split_img = image[row*cam_pixel_rows:(row+1)*cam_pixel_rows, column*cam_pixel_columns:(column+1)*cam_pixel_columns:, :]
        
        split_img = cv2.cvtColor(split_img, cv2.COLOR_BGRA2BGR) 
        
        result.append(split_img)
        
    return result



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Uses OpenPose to calibrate camera intrinsics')
    parser.add_argument('-vy','--video_rows', help='The amount of cameras in the vertical axis', required=False, default=2, type=int)
    parser.add_argument('-vx','--video_columns', help='The amount of cameras in the horizontal axis', required=False, default=2, type=int)
    parser.add_argument('-cc','--cam_count', help='The amount of cameras', required=False, default=2, type=int)
    args = vars(parser.parse_args())
    
    script_dir = Path(__file__).parent.resolve()
    openpose_dir = (script_dir/'../openpose').resolve()
    openpose_python_dir = (openpose_dir/'build/python').resolve()

    sys.path.append(str(openpose_python_dir))
    from openpose import pyopenpose as op

    # Get the calibration matrices and map them to camera id
    calibration_dir = script_dir/'calibration/final'
    calibration_matrices = dict()
    for calibration_xml in calibration_dir.iterdir():
        cv_file = cv2.FileStorage(str(calibration_xml), cv2.FILE_STORAGE_READ)
        extrinsics = cv_file.getNode("CameraMatrix").mat()
        intrinsics = cv_file.getNode("Intrinsics").mat()
        cv_file.release()
        
        calibration_matrices.update({int(calibration_xml.stem): np.matmul(intrinsics, extrinsics)})


    ## OpenPose
    # Custom Params (refer to include/openpose/flags.hpp for more parameters)
    op_params = dict()
    op_params["model_folder"] = str(openpose_dir/'models')
    op_params["face"] = False
    op_params["hand"] = False
    op_params["3d"] = True
    op_params["frame_undistort"] = True
    op_params["number_people_max"] = 1
    op_params["camera_parameter_path"] = str(calibration_dir)
    op_params["3d_min_views"] = -1
    op_params["3d_views"] = args['cam_count']

    # Starting OpenPose
    op_wrapper = op.WrapperPython()
    op_wrapper.configure(op_params)
    op_wrapper.start()

    ## NDI stream setup
    if not ndi.initialize():
        raise Exception("NDI failed to initialize")

    ndi_find = ndi.find_create_v2()

    if ndi_find is None:
        raise Exception("Could not create NDI finder")

    # Look for OBS stream source
    source = None
    while source == None:
        print('Looking for OBS source...')
        ndi.find_wait_for_sources(ndi_find, 1000)
        sources = ndi.find_get_current_sources(ndi_find)
        source = next(filter(lambda x: 'OBS' in x.ndi_name, sources), None)
        
    # Initialize NDI source
    ndi_receiver_create = ndi.RecvCreateV3()
    ndi_receiver_create.color_format = ndi.RECV_COLOR_FORMAT_BGRX_BGRA

    ndi_receiver = ndi.recv_create_v3(ndi_receiver_create)

    if ndi_receiver is None:
        raise Exception("Failed to create NDI receiver")
    
    ndi.recv_connect(ndi_receiver, source)
    ndi.find_destroy(ndi_find)

    pose2twin = Pose2TwinServer()
    pose2twin.connect_to_mqtt('192.168.0.11')

    # Loop while listening for NDI images
    while True:
        type, image_grid_src, audio_src, _ = ndi.recv_capture_v2(ndi_receiver, 5000)

        if type == ndi.FRAME_TYPE_AUDIO:
            ndi.recv_free_audio_v2(ndi_receiver, audio_src)
            
        if type == ndi.FRAME_TYPE_VIDEO:
            # Retrieve image and split it into one image per camera
            image_grid = np.copy(image_grid_src.data)
            images = split_image(image_grid, args['video_rows'], args['video_columns'], args['cam_count'])
            
            # Prepare images and their respective camera matrices for OpenPose
            datums = []
            for cam_id in range(args['cam_count']):
                datum = op.Datum()
                datum.cvInputData = images[cam_id]
                datum.cameraMatrix = calibration_matrices.get(cam_id)
                datum.subId = cam_id
                datum.subIdMax = args['cam_count'] - 1
                datums.append(datum)
            
            op_wrapper.emplaceAndPop(op.VectorDatum(datums))
            
            # Format data for twin2pose server
            if datums[0].poseKeypoints3D is not None:
                openpose_bones = datums[0].poseKeypoints3D[0]
            
                if openpose_bones is not None:
                    twin2pose_data = dict()
                    for joint_type in JointType:
                        twin2pose_data.update({joint_type.name: openpose_bones[joint_type]})
                    pose2twin.submit_human_joint_positions(twin2pose_data)
                
            # if openpose_bones is not None:
            #     pass
            
            ndi.recv_free_video_v2(ndi_receiver, image_grid_src)

        if cv2.waitKey(1) & 0xff == 27:
            break
        
    ndi.recv_destroy(ndi_receiver)
    ndi.destroy()
    cv2.destroyAllWindows()