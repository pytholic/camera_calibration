# Introduction 

This is the library for camera calibration and undistortion. Currently, we support two camera models: fisheye camera and normal camera (pinhole camera), and two types of "sensor": RGB sensor and thermal (infrared) sensor.

The library includes 2 scripts:
- [calibration.py](calibration.py): script to capture the checkerboard and **calibrate** the camera at real-time 
- [undistortion.py](undistortion.py): script to **undistort** the camera at real-time

Utilities:
- [utils.py](calibrator.py): Calibrator class and other utility functions.
- [Checkerboard-A3-55mm-6x4.pdf](Checkerboard-A3-55mm-6x4.pdf): checkerboard pattern file for calibrating the camera
- [environment.yaml](environment.yaml): Anaconda environment file for installing dependencies

# Installation
- Create a new environment using Anaconda:
```conda env create -f environment.yaml```
- Activate the environment:
```conda activate calib```
# Usage

## 1. Calibration

### 1.1. Calibration using stream data (video/USB stream)

1. Print the checkerboard and attach it on a rigid, flat surface (you can use the [existing file](Checkerboard-A3-55mm-6x4.pdf) or download from [here](https://markhedleyjones.com/projects/calibration-checkerboard-collection)). Make sure that there is a thick white border all around the checkerboard pattern. This white border is needed to facilitate the corner detection.

2. Run the script **calibration.py** with the below command:

```
python calibration.py -i [index] -m [camera_model] -t [thermal] -l [low_res] -c [checkerboard_size] -p [param_thres] -q [quantity_thres] -o [output_path] -g [good] -d [detect]
```

where:

- **index**: input source (camera index/video path). The default value is 0.
 - **camera_model**: camera model (0: pinhole, 1: fisheye). The default value is 0 (pinhole camera).
 - **thermal**: flag to set camera is thermal or not. The default value is False (if the camera is thermal, put flag -t into the command, otherwise just skip it).
  - **low_res**: flag to set if the camera/video has a low resolution. The default value is False (if the camera/video has a low resolution, put flag -l into the command, otherwise just skip it). If the camera/video has a considerably good resolution, it is highly recommended to skip this flag to achieve a better checkerboard detection result => better calibration result.
 - **checkerboard_size**: checkerboard size (cols x rows). The default value is 6, 4.
 - **param_thres**: parameter threshold for evaluating the detected corners are "good" and will be used for calibration. The default value is 0.2.
 - **quantity_thres**: the minimum number of images with good detected corners that will be considered enough for calibration. The default value is 20 (images).
 - **output_path**: path to the output calibration file (DAT file). Default path is 'output.dat'.
 - **good**: directory to save "good" images (images that have good detected corner). Leave it empty if you don't want to save.
 - **detect**: directory to save "detected" images (good corner detection result). Leave it empty if you don't want to save.
 
example:
```
python calibration.py -i 0 -m 1 -l -c "10,7" -o "C:\Users\hoa\Pictures\Camera Roll\calibration.dat" -g good -d detect
```

3. Move the camera to capture the checkerboard image in different viewpoints. Try to to cover all the visible area of the camera and make sure that **every corner of the checkerboard is visible** from the camera viewpoint. On each viewpoint where checkerboard corners are detected, these corners will be displayed:
```
  - If the script didn't detect corners in the current image, a message "Detect corners failed. Accumulated [number of collected images[] images!" will be displayed
  
  - If the script detected corners in the current image, but the detected corners aren't "good" enough to be used for the calibration, a message "Detect bad corners. Accumulated [number of collected images[] images!" will be displayed

  - If the script detected corners in the current image and the detected corners are "good" so that the image can be used for the calibration, a message "Detect good corners. Accumulated [number of collected images[] images!" will be displayed
    
  - If there are enough valid images for calibration, a message "Enough images for calibration!" will be displayed
```

To calibrate the thermal (infrared) camera, the checkerboard must be put into the hot surface. 

4. When the script has already collected enough valid images for calibration, press Q on keyboard to finish inspecting images. It will automatically finish if the input is a video stream.
5. The calibration information will be displayed on the terminal.

### 1.2. Calibration using captured images

1. Run the script **calibration_images.py** with the below command:

```
python calibration_images.py -i [input] -m [camera_model] -t [thermal] -l [low_res] -c [checkerboard_size] -p [param_thres] -q [quantity_thres] -o [output_path] -g [good] -d [detect]
```

where:

- **input**: the directory which contains captured images.
 - **camera_model**: camera model (0: pinhole, 1: fisheye). The default value is 0 (pinhole camera).
 - **thermal**: flag to set camera is thermal or not. The default value is False (if the camera is thermal, put flag -t into the command, otherwise just skip it).
  - **low_res**: flag to set if the camera/video has a low resolution. The default value is False (if the camera/video has a low resolution, put flag -l into the command, otherwise just skip it). If the camera/video has a considerably good resolution, it is highly recommended to skip this flag to achieve a better checkerboard detection result => better calibration result.
 - **checkerboard_size**: checkerboard size (cols x rows). The default value is 6, 4.
 - **param_thres**: parameter threshold for evaluating the detected corners are "good" and will be used for calibration. The default value is 0.2.
 - **quantity_thres**: the minimum number of images with good detected corners that will be considered enough for calibration. The default value is 20 (images).
 - **output_path**: path to the output calibration file (DAT file). Default path is 'output.dat'.
 - **good**: directory to save "good" images (images that have good detected corner). Leave it empty if you don't want to save.
 - **detect**: directory to save "detected" images (good corner detection result). Leave it empty if you don't want to save.
 
example:
```
python calibration_images.py -i input -m 1 -l -c "10,7" -o "C:\Users\hoa\Pictures\Camera Roll\calibration.dat" -g good -d detect
```

3. On each image where checkerboard corners are detected, these corners will be displayed:
```
  - If the script didn't detect corners in the current image, a message "Detect corners failed. Accumulated [number of collected images[] images!" will be displayed
  
  - If the script detected corners in the current image, but the detected corners aren't "good" enough to be used for the calibration, a message "Detect bad corners. Accumulated [number of collected images[] images!" will be displayed

  - If the script detected corners in the current image and the detected corners are "good" so that the image can be used for the calibration, a message "Detect good corners. Accumulated [number of collected images[] images!" will be displayed
    
  - If there are enough valid images for calibration, a message "Enough images for calibration!" will be displayed
```

To calibrate the thermal (infrared) camera, the checkerboard must be put into the hot surface. 

4. When the script has already collected enough valid images for calibration, press Q on keyboard to finish inspecting images. It will automatically finish when every image has already been inspected.
5. The calibration information will be displayed on the terminal.

## 2. Undistortion
1. Run the script **undistortion.py** with the below command:

```
python undistortion.py -i [index] -m [camera_model] -c [calibration_path] -s [show_original] -z [zoom]
```
where:
- **index**: input source (camera index/video path). The default value is 0.
 - **camera_model**: camera model (0: pinhole, 1: fisheye). The default value is 0 (pinhole camera).
- **calibration_path**: path to the OpenCV calibration file (DAT file).
- **show_original**: show original video or not (set as 0 if you don't want to show original video, otherwise set as 1). Default value is 0.
- **zoom**: Zoom value, range from 0 (zoomed in, all pixels in calibrated image are valid) to 1, (zoomed out, all pixels in original image are in calibrated image). Default value is 0.

example:
```
python undistortion.py -i 0 -m 1 -c "C:\Users\hoa\Pictures\Camera Roll\calibration.dat" -s 1 -z 1.0
```
2. Press Q on the keyboard to exit the stream.

*Note: Until now, the library seemingly cannot undistort the entire fisheye image: the region around the image border will be cropped out even if you set zoom value as 1*

# Credit
The library is mostly based on [ROS calibration library](https://github.com/ros-perception/image_pipeline/tree/noetic/camera_calibration) 
