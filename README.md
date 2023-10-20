# Apple Trees Skeletonization and Characterization

This repo contains the implementation of a geometrical-based 3D tree architecture traits characterization pipeline. The entire pipeline consists of a structure aware point cloud downsampling, a skeletonization [ROSA](https://github.com/jjcao/skeletonization), and architecture traits characterization algorithm.

## Hilbert Curve Downsampling

A sturcture-aware method to downsample apple tree pointcloud data.

### hilbertcurve

A python file used to generate hilbert curve object.

#### Parameters

- The order of hilbert curve $p$
- The dimension of hibert curve $n$. default: $n=3$

### Hilbertcurve method

A downsampling method for pointcloud using hibert curve.

#### Parameters

- The order of hilbert curve $P$
- The bin size of a histogram bin_size
- The number of points after downsampling subsample_num
- pointcloud data $pc$

## Skeletonization

### Geometric contraction

A laplacian-based contraction method iteratively contracts the point cloud.

- Diagonal length of the point cloud as a cube *diameter*
- A laplacian matrix *L* computed via 1-ring of a Delaunary triangulation
- Diagonal matrices *W<sub>L</sub>* and *W<sub>H</sub>* balance the contraction and attraction
- A scale factor *S<sub>L</sub>*

### Topological thinning

Containing Farthest point sampling (FPS), edge connectivity, and edge collapse.

- FPS resolution &epsilon; (i.e., sample radius)
  - Derived from *diameter*

## Characterization

Point cloud density was used to approximate the thickness of the trunk and branches. The density of each point cloud was computed in **`compute_density.m`**. Then during **FPS** and **Edge_Collapse**, the density was manipulated. A biology aware graph-based tree structure segmentation algorithm and geometric-based tree architecture traits chracterization.

### Biology Aware Graph and MST

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

