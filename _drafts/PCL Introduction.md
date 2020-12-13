# Point Cloud Library

> The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D **image** and **point cloud** processing.

点云库（PCL）的主要release的模块包括：

- Filters
- Features
- Keypoints
- Registration
- KDTree
- Octree
- Segmentation
- Sample-Consensus
- Surface
- Recognition
- IO
- Visualization

点云库的官方文档分为三个部分：

- 点云库的使用教程 ，即Tutorials部分 (https://pcl.readthedocs.io/projects/tutorials/en/latest/)。这部分由多位作者的博客组成，每篇博客提供了PCL模块的使用样例。
- 点云库的内部实现，即Advanced Topics部分 (https://pcl.readthedocs.io/projects/advanced/en/latest/)
- 点云库的参考API，即API Reference部分 (https://pointclouds.org/documentation/)

点云的使用教程部分的文档是比较重要的，这部分文档给出了每个模块的使用样例，但是如果想要对某个模块有更深入的了解，则需要自己去研读参考API的使用说明。

现在最新的点云库的版本是**1.11**.

## 安装

在使用和学习PCL之前，我们首先要安装PCL。如果是在Ubuntu 16.04里用PCL，安装起来比较麻烦。我现在用的是Ubuntu 18.04，PCL已经集成到Ubuntu 18.04的包管理工具中了。所以我们要安装PCL，只需要

```shell
sudo apt install libpcl-dev
```

## 基本介绍

### Filters

因为测量误差(measurement errors)的存在，我们的点云数据集中的数据一般都会有一些outlier。这些outliers在点云中被称为shadow points。这些shadow points会使我们的点云3D特征估计变得复杂，同时影响3D特征估计的性能。我们需要对点云数据做预处理来去掉这些shadow points。

我们可以使用统计分析(statistical analysis)来去掉一部分outliers。一般方法是

> For each point, the **mean distance** from it to all its neighbors is computed. By assuming that the resulting distribution is Gaussian with a mean and a standard deviation, all points whose mean distances are outside an interval defined by the **global distances mean and standard deviation** can be considered as outliers and trimmed from the dataset.

### Features

这个模块包含了点云数据的3D特征提取机制和数据结构。3D特征描述了某个点的几何特征，该点周围的空间被称为k-neighborhood。两个广泛使用的点的几何特征是:

- 几何体表面的curvature
- 几何体表面的norm

这两个几何特征都是局部特征，它们只使用了某个点周围的k closest neighbor。

为了更有效的搜索点的neighbor，我们一般要把输入点云数据分成小块。常用的方法是使用Octree或者KDTree，然后在分割好的空间中，搜索neighbor。在选取neighbor的时候，我们可以选择选取固定K个neighbor，也可以选择固定半径R内的neighbor。

### Keypoints

这个模块实现了两种关键点检测算法。

> Keypoints (also referred to as interest points) are points in an image or point cloud that are stable, distinctive, and can be identified using a well-defined detection criterion.

一般关键点的数量比点云中点的数量少得多。类似与图像中的关键点，我们可以用关键点和描述子提供一种对原始数据的紧凑表述。

### Registration

Registration是将多个点云数据集合并为一个全局一致的数据集。主要思想是找到多个数据集中的对应点，同时找到一个最小化对应点之间距离(alignment error)的几何变换(transformation)。

>  This process is repeated, since **correspondence search is affected by the relative position and orientation of the data sets**. Once the alignment errors fall below a given threshold, the registration is said to be complete.



### KD-Tree

KD-Tree模块提供了KDTree数据结构，我们可以快速搜索最近邻。

> A [Kd-tree](http://en.wikipedia.org/wiki/Kd-tree) (k-dimensional tree) is a space-partitioning data structure that stores a set of k-dimensional points in a tree structure that enables efficient range searches and nearest neighbor searches.

### Octree

Octree模块提供了用于创建hierarchical tree data structure的有效方法。我们可以用它来划分空间，降采样和搜索。

- 每个octree节点有八个或者零个子节点
- 根节点是一个包括所有点的立方体
- 在树的每一层，空间会被二次划分为八个子空间

Octree实现了高效地最近邻搜索算法。

### Segmentation

Segmentation模块包含了用于将点云segment成多个cluster的算法。

### Sample Consensus

Sample Consensus模块包含了SAC方法，比如RANSAC和一些集合模型。

### Surface

Surface模块包含了重建物体表面的算法。

### Range Image

Range image模块包含了两个类用来表示和处理range images。

> A range image (or depth map) is an image whose pixel values represent a distance or depth from the sensor’s origin. 

Range image是一种常用的用来表示3D数据的方法，通常由stereo或者time-of-flight摄像头生成。如果知道摄像头的内参，我们可以把range Image转换成点云。

### IO

IO模块主要是用来读写点云数据文件(PCD)，而且可以从传感器读取点云数据。

### Visualization

Visualization模块主要是用来展示点云数据和点云算法的结果。

### Common

Common提供了常用的数据结构和算法。

### Search

Search模块提供了使用不同数据结构搜索最近邻的算法。

### Binaries

Binaries提供了一些点云库中常用的工具

- pcl_viewer
- pcd_convert_NaN_nan
- convert_pcd_ascii_binary
- concatenate_points_pcd
- pcd2vtk
- pcd2ply
- mesh2pcd
- octree_viewer

