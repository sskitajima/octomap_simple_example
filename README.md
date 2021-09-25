# Simple Octoamp example

- How to use
```bash
mkdir build
cd build
cmake ..
make
./octomap_simple_example [path_to_depth_img]
```

- execution result
  - `octomap.bt`...octomap binary file
  - `pointx.txt`...3d position of points recovered from the depth image
- parameter
  - camera intrinsic parameter
    - fx
    - fy
    - cx
    - cy
    - size of the image
  - octomap parameter
    - prob_hit
    - prob_miss
    - occupancy_thres
    - voxel_size
  - others
    - depth_min and depth_max...for removing outlier depth measurement
    - depthmap_factor...for acquiring real depth value from pixel value of the depth image