# AppleQSM

This repo contains the implementation of a 3D geometry-based apple tree characterization pipeline. The entire pipeline consists of a skeletonization [ROSA](https://github.com/jjcao/skeletonization), a trunk and branch segmentation, and a trait characterization method.

## Workflow

![alt text](./cover/pipeline.png)

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