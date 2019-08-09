# NuScenes Lidar Visualized in PCL Visualizer

Quick and simple visualizer of NuScenes LIDAR_TOP sweeps in PCL. Tested on Ubuntu 18.04.

## Build:
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
