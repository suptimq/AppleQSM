# Apple Trees Skeletonization and Characterization

This repo contains the implementation of a 3D geometry-based apple tree characterization pipeline. The entire pipeline consists of a skeletonization [ROSA](https://github.com/jjcao/skeletonization), a trunk and branch segmentation, and a trait characterization method.

## Requirements

- MATLAB 2022a or higher
  - [Computer Vision ToolBox](https://www.mathworks.com/products/computer-vision.html)
  - [Statistical and Machine Learning ToolBox](https://www.mathworks.com/products/statistics.html)
  - [Optimization ToolBox](https://www.mathworks.com/products/optimization.html)
  - [Curve Fitting ToolBox](https://www.mathworks.com/products/curvefitting.html)
  - [Parallel Computing Toolbox](https://www.mathworks.com/products/parallel-computing.html)
  - [YAML Parser](https://www.mathworks.com/matlabcentral/fileexchange/106765-yaml?s_tid=srchtitle_site_search_1_yaml)
  - [NATSORT Function](https://www.mathworks.com/matlabcentral/fileexchange/47433-natural-order-row-sort?s_tid=ta_fx_results)
  - [Point-to-Line-Distance Function](https://www.mathworks.com/matlabcentral/fileexchange/64396-point-to-line-distance?s_tid=srchtitle_support_results_1_point_to_line)
  - [Line-Line Intersection](https://www.mathworks.com/matlabcentral/fileexchange/59805-line-line-intersection-n-lines-d-space)
  - [matGeom](https://www.mathworks.com/matlabcentral/fileexchange/107370-matgeom)
  - [Configuration Python in MATLAB](https://www.mathworks.com/help/matlab/matlab_external/install-supported-python-implementation.html)
- Python 3.8
  - numpy>=1.23.4
  - [scikit-image>=0.19.3](https://scikit-image.org/docs/stable/user_guide/install.html)
  - [pyransac3d](https://pypi.org/project/pyransac3d/)

## Configuration

All parameters associated with the pipeline should be defined in a **.yaml** file saved in `config`. All interests of **.pcd** files are supposed to be saved in the `data_folder/model/mode` directory. If you don't have particular models to generate the **.pcd** files, you can leave `models` and `mode` as `.`, like showing below.


```
experiment: {
  models: ['.'],              # ['model1', 'model2']
  mode: .,                    # customization
  num_tree: 9,
  exp_folder: AppleQSM,
  exp_name: ,                       # leave it empty if you dont want a specific one
  skeleton_folder: Skeleton,
  segmentation_folder: Segmentation,
  characterization_folder: Characterization,
  data_folder: E:\Result\LLC_02022022\Row13,
  pcd_extension: .pcd,
  mat_extension: .mat,
  # switch (suggested to turn on one-by-one)
  SKEL_ON: False,                     # turn on skeletonization
  SEG_ON: False,                     # turn on segmentation
  CHAR_ON: False,                    # turn on characterization
}

```

## Usage

The entry file is `characterizer.m`. Please read this file in detail if it is the first time you are using **AppleQSM**. The code should be easy to understand without going into detailed functions. The above `SKEL_ON`, `SEG_ON`, and `CHAR_ON` are to turn on the **tree skeletonization**, **tree structure segmentation (i.e., trunk/branch instance segmentation)**, and **trait extraction**. It is recommended to only leave one of them on so you can check the stage-wise results before moving forward.


## Skeletonization

### Hilbert Curve Downsampling

A sturcture-aware method to downsample apple tree pointcloud data. Hilbertcurve_method use space-filling curve to map a 3D coordinate to `[0,1]`.

#### Parameters

- **num_iteration**: The order of hilbert curve 
- **bin_size**: The bin size of a histogram
- **downsample_num**: The number of points after downsampling subsample_num

### Geometric contraction

A laplacian-based contraction method iteratively contracts the point cloud.

#### Parameters

- Diagonal length of the point cloud as a cube *diameter*
- A laplacian matrix *L* computed via 1-ring of a Delaunary triangulation
- Diagonal matrices *W<sub>L</sub>* and *W<sub>H</sub>* balance the contraction and attraction
- A scale factor *S<sub>L</sub>*

### Topological thinning

Containing Farthest point sampling (FPS), edge connectivity, and edge collapse.

#### Parameters

- FPS resolution &epsilon; (i.e., sample radius)
  - Derived from *diameter*

## Segmentation

Point cloud density was used to approximate the thickness of the trunk and branches. The density of each point cloud was computed in **`compute_density.m`**. Then during **FPS** and **Edge_Collapse**, the density was manipulated. A biology aware graph-based tree structure segmentation algorithm and geometric-based tree architecture traits chracterization.

### Biology-Aware Graph and MST

Develop a good MST that provides as much accurate connection as possible. The coefficient controls the ratio of local region thickness and length information to the edge weight.

- Graph adjacency matrix 
  - **distance threshold** - &lambda;<sub>1</sub> 
  - **refinement mode**
- Graph edge weight
  - **coefficient** - &alpha;<sub>1</sub>
- Root point 
  - **search range** - &epsilon; with small deviation

### Trunk Segmentation

Segment the trunk by finding the maximum path starting from the root point.

- Graph edge weight
  - **coefficient** - &alpha;<sub>2</sub>
- Trunk refinement 
  - **search range** - &epsilon; with small deviation

### Trunk Diameter Estimation

Estimate the trunk diameter by 2D RANSAC ellipse fitting algorithm.

- Point slicing along Z-axis
  - **trunk slice range**
- Ellipse fitting with RANSAC
  - **minimum samples**
  - **residual threshold** - typicall small ~5mm
  - **max trial**

### Branch Segmentation

Segment individual branch in a similar way to the trunk segmentation.

- Circle pruning
  - **circle radius** - *X* times of &epsilon;
- DBSCAN clustering
  - **eps**
  - **minimum samples**
- Sub-Graph adjacency matrix 
  - **distance threshold** - &lambda;<sub>2</sub> 
  - **refinement mode**
- Sub-Graph edge weight
  - **coefficient** - &alpha;<sub>3</sub>
- Internode detection
  - **trunk slice range**
- Primary branch detection

### Branch Post-Processing

A projection distance check is applied on all segmented branches to remove fake branch origin clusters.

- Projection distance threshold
  - **line_distance_threshold**
- Point distance threshold
  - **point_distance_threshold**

### Angle Descriptor

Produce a trunk angle descriptor, a branch vertical angle descriptor, and a branch horizontal angle descriptor. The branch vertical angle descriptor is the vertical angle between the 1st segment of the branch and the sliced trunk, whereas the branch horizontal angle descriptor is the horizontal angle between the 1st segment of the branch and the planting line.

- Planting line fitting
  - 3D RANSAC line fitting
- Trunk segment computation (i.e., sliding window fashion)
  - **sliding window length/stride**
  - 3D RANSAC line fitting
- Branch segment computation (i.e., sliding window fashion)
  - **sliding window length/stride**
  - 3D RANSAC line fitting

### Branch Diameter Estimation

Estimate the diameter of branches by a voting method that consists of a 3D circle fitting, 3D cylinder fitting w/ and w/i reference vector. Prior to the estimation, there is a critical step to retrieve as many branch points as possible (most likely bu haven't confirmed that). The branch surface points are found and a sphere is used to retrieve neighbors. A KD-Tree is built upon the original point cloud (i.e., w/i downsampled) for the further retrieval, which brings in trivial improvement experimentally.

- KD-Tree
  - **#neighbors**
- Branch radius bounds
  - **lower bound** and **upper bound**
- Branch point retrieval
  - **#candidate points** - typicall **1**
  - **sphere radius** - divide trunk radius by *X* times in **`retrieve_points.m`**
- Branch radius estimation
  - 3D RANSAC circle fitting
  - 3D cylinder fitting w/i reference vector
  - 3D cylinder fitting w/ reference vector
    - **maximum distance**
    - **reference vector** - typically the 1st branch segmention direction
    - **maximum angular distance**

-------------------------------------------------------

If you find our code helpful for your research, please cite

```
@article{qiu2024appleqsm,
  title={AppleQSM: Geometry-Based 3D Characterization of Apple Tree Architecture in Orchards},
  author={Qiu, Tian and Wang, Tao and Han, Tao and Kuehn, Kaspar and Cheng, Lailiang and Meng, Cheng and Xu, Xiangtao and Xu, Kenong and Jiang, Yu},
  journal={Plant Phenomics},
  year={2024},
  publisher={AAAS}
}
```

Please contact me (tq42@cornell.edu) if you have any questions!