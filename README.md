# NuScenes Lidar Visualized in PCL Visualizer

Quick and simple visualizer of NuScenes LIDAR_TOP sweeps in C++ with PCL. 

## Dependencies
Tested on Ubuntu 18.04.

Required system libs:
- JsonCpp
- YamlCpp
- PCL 

## Build
CMake based:
```
mkdir build
cd build
cmake ..
make
```

## Run
Edit `config.yaml` to point to your [NuScenes dataset](https://www.nuscenes.org/). (WARNING: It's definitely slow for full NuScenes dataset but works good for `v1.0-mini` sizes).
```
./nuscenes_pcl_viz ../config.yaml
```
Press `n` to switch between scenes. And default `h` key for help (PCLVisualizer).
