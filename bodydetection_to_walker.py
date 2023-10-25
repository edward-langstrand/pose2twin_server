import sys
import numpy as np
import cv2 as cv
import NDIlib as ndi
import inspect
import math

import carla
import random

sys.path.append('/usr/local/python')
from openpose import pyopenpose as op

cam0_intrinsics = [[1.8454899023950436e+03, 0., 9.4906292579387718e+02], 
                   [0., 1.8478155481950039e+03, 6.3022852011023485e+02], 
                   [0., 0., 1.]]

cam0_extrinsics = [[1., 0., 0., 0.], 
                   [0., 1., 0., 0.], 
                   [0., 0., 1., 0.]]

cam1_intrinsics = [[3.3773586603245094e+03, 0., 6.6272738561910694e+02], 
                   [0., 3.4126290181578065e+03, 5.9968673379586119e+02], 
                   [0., 0., 1.]]

cam1_extrinsics = [[5.3826610712985756e-01, 2.2886761862148070e-02, -8.4246412033197848e-01, 3.0587904052814334e+00], 
                    [-2.3837746824726027e-02, 9.9964469807429035e-01, 1.1926417663381884e-02, -3.2172479929960987e-02], 
                    [8.4243774828861340e-01, 1.3662860001715882e-02, 5.3862042898035767e-01, 3.6839454219514161e+00]]

cam0_matrix = np.matmul(cam0_intrinsics, cam0_extrinsics)
cam1_matrix = np.matmul(cam1_intrinsics, cam1_extrinsics)

# cam0_matrix = np.transpose(cam0_matrix)
# cam1_matrix = np.transpose(cam1_matrix)


# Custom Params (refer to include/openpose/flags.hpp for more parameters)
params = dict()
params["model_folder"] = "./models/"
params["face"] = False
params["hand"] = False
params["3d"] = True
# params["flir_camera"] = False
params["frame_undistort"] = True
params["number_people_max"] = 1
params["camera_parameter_path"] = "./models/cameraParameters/"
params["3d_min_views"] = 2
params["3d_views"] = 2

# Starting OpenPose
opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

if not ndi.initialize():
    raise Exception("NDI failed to initialize")

ndi_find = ndi.find_create_v2()

if ndi_find is None:
    raise Exception("Could not create NDI finder")

sources = []
while not len(sources) > 1:
    print('Looking for sources ...')
    ndi.find_wait_for_sources(ndi_find, 1000)
    sources = ndi.find_get_current_sources(ndi_find)
    
ndi_receiver_create = ndi.RecvCreateV3()
ndi_receiver_create.color_format = ndi.RECV_COLOR_FORMAT_BGRX_BGRA

ndi_receiver_create2 = ndi.RecvCreateV3()
ndi_receiver_create2.color_format = ndi.RECV_COLOR_FORMAT_BGRX_BGRA

ndi_receiver = ndi.recv_create_v3(ndi_receiver_create)
ndi_receiver2 = ndi.recv_create_v3(ndi_receiver_create2)

if ndi_receiver is None or ndi_receiver2 is None:
    raise Exception("Failed to create NDI receiver")

ndi.recv_connect(ndi_receiver, sources[0])
ndi.recv_connect(ndi_receiver2, sources[1])

ndi.find_destroy(ndi_find)

cv.startWindowThread()


#carla
client = carla.Client('localhost', 2000)
client.set_timeout(2)

world = client.get_world()
blueprint = world.get_blueprint_library().find('walker.pedestrian.0002')
spawn_point = carla.Transform(carla.Location(x=-78.6,y=-107,z=1.6),carla.Rotation(0,90,0))
walker = world.try_spawn_actor(blueprint, spawn_point)

camera_origin = [-78.400, -111.500, 1.0]

bone_index_map = {
    'crl_hips__C': 8, 
    'crl_spine__C': 8,    
    'crl_Head__C': 0, 
    'crl_spine01__C': 1,
     
    'crl_shoulder__R': 1, 
    'crl_arm__R': 2,
    'crl_foreArm__R': 3,
    'crl_hand__R': 4, 
    
    'crl_shoulder__L': 1, 
    'crl_arm__L': 5, 
    'crl_foreArm__L': 6, 
    'crl_hand__L': 7, 
    
    'crl_thigh__R' : 9, 
    'crl_leg__R': 10, 
    'crl_foot__R': 11, 
    
    'crl_thigh__L' : 12, 
    'crl_leg__L': 13, 
    'crl_foot__L': 14
}
bone_target_map = {
    'crl_spine01__C': 'crl_Head__C',
    'crl_spine__C': 'crl_spine01__C',

    'crl_thigh__R': 'crl_leg__R',
    'crl_leg__R': 'crl_foot__R',
    
    'crl_thigh__L': 'crl_leg__L',
    'crl_leg__L': 'crl_foot__L',
    
    'crl_shoulder__R': 'crl_arm__R',
    'crl_arm__R': 'crl_foreArm__R',
    'crl_foreArm__R': 'crl_hand__R', 
    
    'crl_shoulder__L': 'crl_arm__L',
    'crl_arm__L': 'crl_foreArm__L',
    'crl_foreArm__L': 'crl_hand__L', 
}
bone_parent_map = {
    'crl_spine__C': 'crl_hips__C',
    'crl_spine01__C': 'crl_spine__C',
    'crl_Head__C': 'crl_spine01__C',
    
    'crl_shoulder__R': 'crl_spine01__C',
    'crl_arm__R': 'crl_shoulder__R',
    'crl_foreArm__R': 'crl_arm__R',
    'crl_hand__R': 'crl_foreArm__R',
    
    'crl_shoulder__L': 'crl_spine01__C',
    'crl_arm__L': 'crl_shoulder__L',
    'crl_foreArm__R': 'crl_arm__L',
    'crl_hand__R': 'crl_foreArm__R',
    
    'crl_thigh__R': 'crl_hips__C',
    'crl_leg__R': 'crl_thigh__R' ,
    'crl_foot__R': 'crl_leg__R',
    
    'crl_thigh__L': 'crl_hips__C',
    'crl_leg__L': 'crl_thigh__L' ,
    'crl_foot__L': 'crl_leg__L'
}

video_cam1 = None 
video_cam2 = None

def look_at(center, target, up):
    # from https://stackoverflow.com/questions/54897009/look-at-function-returns-a-view-matrix-with-wrong-forward-position-python-im
    # print(center, target  , up)
    
    f = (target - center); f = f/np.linalg.norm(f)
    s = np.cross(f, up); s = s/np.linalg.norm(s)
    u = np.cross(s, f); u = u/np.linalg.norm(u)

    m = np.zeros((4, 4))
    m[0, :-1] = s
    m[1, :-1] = u
    m[2, :-1] = f
    # m[:-1, 3] = center
    m[3, 3] = 1.0

    return m


def get_openpose_location(openpose_bones, bone_key):
    op = openpose_bones[0][bone_index_map[bone_key]]

    return np.array([op[0], op[2], -op[1]])
    

def global_bone_matrix(openpose_bones, bone_key, forward):
    if bone_key not in bone_target_map:
        # this bone has no target, return identity matrix
        # print("identity: " + bone_key)
        return np.identity(4)
    
    center = get_openpose_location(openpose_bones, bone_key)
    target = get_openpose_location(openpose_bones, bone_target_map[bone_key])
    
    
    # up = np.array([0.0, 0.0, 1.0])
    
    return look_at(center, target, forward)


def relative_bone_matrix(openpose_bones, bone_key, forward):
    bone_global = global_bone_matrix(openpose_bones, bone_key, forward)
    
    if bone_key not in bone_parent_map:
        # this bone has no parent, therefore global is relative
        return bone_global
    
    parent_global_inverse = np.linalg.inv(global_bone_matrix(openpose_bones, bone_parent_map[bone_key], forward))
    return np.matmul(bone_global, parent_global_inverse)


def rotation_matrix_from_euler(euler):
    # x_rot = euler.roll
    # y_rot = euler.pitch
    # z_rot = euler.yaw
    
    x_rot = math.radians(euler.pitch)
    y_rot = math.radians(euler.roll)
    z_rot = math.radians(euler.yaw)
    
    R_x = np.array([
        [1,0,0,0],
        [0,math.cos(x_rot),-math.sin(x_rot),0],
        [0,math.sin(x_rot),math.cos(x_rot),0],
        [0,0,0,1]
    ])
    R_y = np.array([
        [math.cos(y_rot),0,math.sin(y_rot),0],
        [0,1,0,0],
        [-math.sin(y_rot),0,math.cos(y_rot),0],
        [0,0,0,1]
    ])
    R_z = np.array([
        [math.cos(z_rot),-math.sin(z_rot),0,0],
        [math.sin(z_rot),math.cos(z_rot),0,0],
        [0,0,1,0],
        [0,0,0,1]
    ])
    
    # R_z = np.transpose(R_z)
    # R_y = np.transpose(R_y)
    # R_x = np.transpose(R_x)
    
    # ZYX rotation order
    return np.matmul(np.matmul(R_z, R_y), R_x)


def extract_rotation(m):
    return carla.Rotation(
        yaw=math.degrees(math.atan2(m[1,0],m[0,0])),
        roll=math.degrees(math.asin(-m[2,1])),
        pitch=math.degrees(math.atan2(m[2,1],m[2,2]))
    )
    

def extract_location(m):
    return carla.Location(
        x=m[0,3],
        y=m[1,3],
        z=m[2,3]
    )


def extract_bones_from_walker(walker):
    result = {}
    
    walker_bones = walker.get_bones()
    
    for old in walker_bones.bone_transforms:
        result.update({old.name: rotation_matrix_from_euler(old.relative.rotation)})

    return result


# def calculate_transform(openpose_bones, bone_key, forward):
#     bone_global = global_bone_matrix(openpose_bones, bone_key, forward)
#     bone_relative = relative_bone_matrix(openpose_bones, bone_key, forward)
    
#     # TEMP: find old bone transform that corresponds to bone_a
#     for old in old_carla_bones.bone_transforms:
#         if old.name == bone_key:
#             old_bone = old.relative
            
#     print('global', extract_rotation(bone_global))
#     print('before', extract_rotation(bone_relative))
    
#     bone_relative = np.matmul(original_bone_transforms[bone_key], bone_relative)
    
#     print('with t-pose', extract_rotation(bone_relative))
    
#     return carla.Transform(
#         rotation=extract_rotation(bone_relative),#extract_rotation(original_bone_transforms[bone_key]),
#         # rotation=old_bone.rotation,
#         # location=extract_location(original_bone_transforms[bone_key]),
#         location=old_bone.location
#     )
    
# def calculate_transform(openpose_bones, bone_a, bone_b, old_carla_bones):
    # # Get openpose bone locations
    # bone_a_op = openpose_bones[0][bone_index_map[bone_a]]
    # bone_b_op = openpose_bones[0][bone_index_map[bone_b]]
    
    # # Convert to carla coordinate system
    # bona_a_carla = np.asarray([-bone_a_op[0], -bone_a_op[2], bone_a_op[1]])
    # bona_b_carla = np.asarray([-bone_b_op[0], -bone_b_op[2], bone_b_op[1]])
    
    # # Get vector from a to b
    # a_to_b = np.subtract(bona_b_carla, bona_a_carla)
    
    # a_to_b_norm = a_to_b / np.linalg.norm(a_to_b)

    # pitch = math.degrees(math.asin(-a_to_b_norm[2]))
    # yaw = math.degrees(math.atan2(a_to_b_norm[1], a_to_b_norm[0]))
    
    # # TEMP: find old bone transform that corresponds to bone_a
    # for old in old_carla_bones.bone_transforms:
    #     if old.name == bone_a:
    #         old_bone = old.relative
            
    # return carla.Transform(
    #     rotation=carla.Rotation(roll=0,pitch=pitch,yaw=yaw),
    #     # rotation=old_bone.rotation,
    #     location=old_bone.location
    #     # locer_poation=carla.Location(x=float(a_to_b[0]),y=float(a_to_b[1]),z=float(a_to_b[2]))
    #     )
    # # return carla.Transform(rotation=carla.Rotation(roll=0,pitch=pitch,yaw=yaw))


# def openpose_to_carla(openpose_bones, old_carla_bones):
#     carla_bones = []
    
#     # early exit if required bones are not tracked
#     for bone_key in ['crl_thigh__R','crl_thigh__L','crl_hips__C','crl_spine01__C']:
#         if openpose_bones[0][bone_index_map[bone_key]][3] <= 0.1:
#             print('not enough bones')
#             return carla_bones
    
#     for bone_key, bone_target in bone_target_map.items():
#         # Check if bones are tracked (high confidence from openpose)
#         if openpose_bones[0][bone_index_map[bone_key]][3] > 0.1 and openpose_bones[0][bone_index_map[bone_target]][3] > 0.1:
#             # if bone_key == 'crl_Head__C':
#             right = get_openpose_location(openpose_bones, 'crl_thigh__R') - get_openpose_location(openpose_bones, 'crl_thigh__L')
#             right = right/np.linalg.norm(right)
#             up = get_openpose_location(openpose_bones, 'crl_hips__C') - get_openpose_location(openpose_bones, 'crl_spine01__C')
#             up = up/np.linalg.norm(up)
            
#             forward = np.cross(up, right)
            
#             bone_transform = calculate_transform(openpose_bones, bone_key, forward)
            
#             carla_bones.append((bone_key, bone_transform))
            
            

    print(str(len(carla_bones)) + " bones")
        
    return carla_bones


original_bone_transforms = extract_bones_from_walker(walker)

# print(original_bone_transforms)



# def turn_vector(vector, angle):
#     rotated_vector = []
    
#     R_x = np.array([
#         [1, 0, 0],
#         [0, math.cos(angle), -math.sin(angle)],
#         [0, math.sin(angle), math.cos(angle)]
#     ])
    
#     R_y = np.array([
#         [math.cos(angle), 0, math.sin(angle)],
#         [0, 1, 0],
#         [-math.sin(angle), 0, math.cos(angle)]
#     ])
    
#     R_z = np.array([
#         [math.cos(angle), -math.sin(angle), 0],
#         [math.sin(angle), math.cos(angle), 0],
#         [0, 0, 1]
#     ])
    
    
#     temp = np.array(vector)
#     rotated_np_vector = R_y.dot(temp)
    
#     for i in rotated_np_vector:
#         rotated_vector.append(i)
    
#     return rotated_vector
    
    

while True:
    type1, video1, audio1, _ = ndi.recv_capture_v2(ndi_receiver, 5000)
    type2, video2, audio2, _ = ndi.recv_capture_v2(ndi_receiver2, 5000)

    if type1 == ndi.FRAME_TYPE_AUDIO:
        ndi.recv_free_audio_v2(ndi_receiver, audio1)
        
    
    if type2 == ndi.FRAME_TYPE_AUDIO:
        ndi.recv_free_audio_v2(ndi_receiver2, audio2)
        

    if type1 == ndi.FRAME_TYPE_VIDEO:
        video_cam1 = video1
        
        
    if type2 == ndi.FRAME_TYPE_VIDEO:
        video_cam2 = video2
        
        
    # print(video_cam2)   
    if video_cam1 != None and video_cam2 != None:    
        frame = np.copy(video_cam1.data)
        frame = cv.cvtColor(frame, cv.COLOR_BGRA2BGR) 
        frame2 = np.copy(video_cam2.data)
        frame2 = cv.cvtColor(frame2, cv.COLOR_BGRA2BGR) 
        
        datum = op.Datum()
        datum.cvInputData = frame
        datum.cameraMatrix = cam0_matrix
        datum.subId = 0
        datum.subIdMax = 1
        
        datum2 = op.Datum()
        datum2.cvInputData = frame2
        datum2.cameraMatrix = cam1_matrix
        datum2.subId = 1
        datum2.subIdMax = 1
        
        vector_dantum = op.VectorDatum([datum, datum2])
        opWrapper.emplaceAndPop(vector_dantum)
        
        openpose_bones = datum.poseKeypoints3D
        
        if openpose_bones is not None:
            # old_carla_bones = walker.get_bones()
            
            # new_carla_bones = openpose_to_carla(openpose_bones, old_carla_bones)
            
            # walker.set_bones(carla.WalkerBoneControlIn(bone_transforms=new_carla_bones))
            
            print(get_openpose_location(openpose_bones, 'crl_hips__C'))
            openpose_hips = get_openpose_location(openpose_bones, 'crl_hips__C')
            real_pos =  camera_origin + get_openpose_location(openpose_bones, 'crl_hips__C')
            walker_pos = walker.get_location()
            
            rotation_angle = math.pi/2
            direction = real_pos - [walker_pos.x, walker_pos.y, walker_pos.z]
            
            direction=carla.Vector3D(direction[0], direction[1], direction[2])
            walker_control = carla.WalkerControl(
                direction = direction,
                speed= direction.length()*6,
                jump=False
            )
            
            walker.apply_control(walker_control)
        
        # Display Image
        # cv.imshow("test", datum2.cvOutputData)#frame)
        # cv.waitKey(0)
        
        ndi.recv_free_video_v2(ndi_receiver, video1)
        ndi.recv_free_video_v2(ndi_receiver2, video2)
        video_cam1 = None 
        video_cam2 = None

    if cv.waitKey(1) & 0xff == 27:
        break
    

walker.destroy()

ndi.recv_destroy(ndi_receiver)
ndi.destroy()
cv.destroyAllWindows()