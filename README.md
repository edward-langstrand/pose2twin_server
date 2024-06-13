# pose2twin_server

## Commands
Make sure to run commands from the root directory of this repository.

Use "--help" (or "-h") to get details for the script arguments.

### OpenPose
Calibrate intrinsics:
```
python3 calibrate_intrinsics.py [-h] -c CAM_ID -v VIDEO [-gss GRID_SQUARE_SIZE_MM] [-gnic GRID_NUMBER_INNER_CORNERS]
```
Calibrate extrinsics:
```
python3 calibrate_extrinsics.py [-h] -c CAM_ORIGIN -v VIDEO [-vc VIDEO_COLUMNS] [-vr VIDEO_ROWS] [-cc CAM_COUNT] [-gss GRID_SQUARE_SIZE_MM] [-gnic GRID_NUMBER_INNER_CORNERS] [-fcp [FORCE_CAMERA_PAIRS [FORCE_CAMERA_PAIRS ...]]]
```
Run server with OpenPose as the tracking system:
```
python3 openpose_server.py [-h] [-vy VIDEO_ROWS] [-vx VIDEO_COLUMNS] [-cc CAM_COUNT]
```

## 3D setup
1. Record one video per camera for intrinsics, here you should move the checkerboard around so that it covers the entire camera view
2. Record one video with the cameras combined into a grid in OBS (in your case 1 row with 2 columns).
3. Calibrate intrinsics, run command for each camera:
  python3 calibrate_intrinsics.py [-h] -c CAM_ID -v VIDEO [-gss GRID_SQUARE_SIZE_MM] [-gnic GRID_NUMBER_INNER_CORNERS]
  VIDEO is the path to your video
  CAM_ID should be 0 for your first and 1 for the seconds
  GRID_SQUARE_SIZE_MM is the size of each checkerboard square in mm
  GRID_NUMBER_INNER_CORNERS is the amount of corners in the checkerboard on each axis (e.g. 6x8)
4. Calibrate extrinsics:
  python3 calibrate_extrinsics.py [-h] -c CAM_ORIGIN -v VIDEO [-vc VIDEO_COLUMNS] [-vr VIDEO_ROWS] [-cc CAM_COUNT] [-gss
  VIDEO is the path to your video
  GRID_SQUARE_SIZE_MM] [-gnic GRID_NUMBER_INNER_CORNERS] [-fcp [FORCE_CAMERA_PAIRS [FORCE_CAMERA_PAIRS ...]]]
  CAM_ORIGIN is the camera that you want to be the origin transform
  VIDEO_COLUMNS/VIDEO_ROWS describes the grid layout of your cameras (2, 1)
  CAM_COUND is the amount of cameras in your grid (2)
  Ignore FORCE_CAMERA_PAIRS (it is relevant if you have more cameras)
6. Run OBS livestream with NDI plugin so that the OpenPose server can retrieve it (name the stream something with OBS and the server will use it). Make sure to use the same grid layout as you did in the extrinsics video.
7. Run the server:
  python3 openpose_server.py [-h] [-vy VIDEO_ROWS] [-vx VIDEO_COLUMNS] [-cc CAM_COUNT]
    Parameters are the same as before
