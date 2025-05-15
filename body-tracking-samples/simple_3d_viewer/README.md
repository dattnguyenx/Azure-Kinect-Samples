# Azure Kinect Body Tracking Simple3dViewer Sample

## Introduction

The Azure Kinect Body Tracking Simple3dViewer sample creates a 3d window that visualizes all the information provided
by the body tracking SDK.

## Usage Info

USAGE: simple_3d_viewer.exe SensorMode[NFOV_UNBINNED, WFOV_BINNED](optional) RuntimeMode[CPU, OFFLINE](optional)
* SensorMode:
  * NFOV_UNBINNED (default) - Narraw Field of View Unbinned Mode [Resolution: 640x576; FOI: 75 degree x 65 degree]
  * WFOV_BINNED             - Wide Field of View Binned Mode [Resolution: 512x512; FOI: 120 degree x 120 degree]
* RuntimeMode:
  * CPU - Use the CPU only mode. It runs on machines without a GPU but it will be much slower
  * OFFLINE - Play a specified file. Does not require Kinect device. Can use with CPU mode

```
e.g.   simple_3d_viewer.exe WFOV_BINNED CPU
                 simple_3d_viewer.exe CPU
                 simple_3d_viewer.exe WFOV_BINNED
                 simple_3d_viewer.exe OFFLINE MyFile.mkv
```

## Instruction

### Basic Navigation:
* Rotate: Rotate the camera by moving the mouse while holding mouse left button
* Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button
* Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel
* Select Center: Center the scene based on a detected joint by right clicking the joint with mouse

### Key Shortcuts
* ESC: quit
* h: help
* b: body visualization mode
* k: 3d window layout


### Additional info for human body extraction and saving depth frame to PNG/point cloud
Additional modules have been added into the original source for: 
* Remove/keep background from the scene
* Compress the modified scene to PNG images or export to point clouds for further compression
Carefully check the main.cpp for details
## Steps:
* Modify the path in main.cpp to match your local files/folders
* Build the program, the easiest way is using Microsoft Visual Studio, the output will be written to simple_3d_viewer.exe

* Start the program
```
cd simple_3d_viewer\build\bin\Debug
.\simple_3d_viewer.exe OFFLINE "C:\Users\ke76boqe\Projects\DepthCompression\test.mkv"
```
* The PNG images will be written into folder ``` output_png_file_name``` and point clouds in .ply format will be written into folder ```point_cloud_file_name```
* The PNG images is already a compressed format, no further step needed
* The .ply point clouds can be encoded using provided source in point_cloud_coding/


## PNG images coding: 

## Point Cloud Coding
* main.py takes 2 paths as input, input_glob is the folder where the .ply can be found and output where the bitstream will be stored
* run: python3 main.py in a python3 environment with numpy, pyntcloud, pickle, math, glob packages