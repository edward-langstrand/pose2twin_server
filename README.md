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
