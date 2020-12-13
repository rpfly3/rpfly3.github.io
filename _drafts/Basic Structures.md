# Basic Structures

PCL中最基本的数据类型是`pcl::PointCloud<pcl::PointT>`， 它包含了成员变量：

## width (`int`)

Specifies the width of the point cloud dataset in the number of points. *width* has two meanings:

1. it can specify the total number of points in the cloud (equal with the number of elements in **points** – see below) for **unorganized** datasets;

2. it can specify the width (total number of points in a row) of an **organized** point cloud dataset.

>  An **organized point cloud** dataset is the name given to point clouds that resemble an organized image (or matrix) like structure, where the data is split into rows and columns. Examples of such point clouds include data coming from stereo cameras or Time Of Flight cameras. The advantages of a organized dataset is that by knowing the relationship between adjacent points (e.g. pixels), nearest neighbor operations are much more efficient, thus speeding up the computation and lowering the costs of certain algorithms in PCL.

An **projectable point cloud** dataset is the name given to point clouds that have a correlation according to a pinhole camera model between the (u,v) index of a **point in the organized point cloud** and the actual 3D values. This correlation can be expressed in it’s easiest form as: u = f*x/z and v = f*y/z

## height (`int`)

Specifies the height of the point cloud dataset in the number of points. *height* has two meanings:

1. it can specify the height (total number of rows) of an organized point cloud dataset;

2. it is set to **1** for unorganized datasets (*thus used to check whether a dataset is organized or not*).

## points (`std::vector<PointT>`)

Contains the data array where all the points of type `PointT` are stored. For example, for a cloud containing XYZ data, **points** contains a vector of `pcl::PointXYZ` elements.

## is_dense (`bool`)

Specifies if all the data in **points** is finite (true), or whether the XYZ values of certain points might contain `Inf/NaN` values (false).

## sensor_origin_ (`Eigen::Vector4f`)

Specifies the sensor acquisition pose (**origin/translation**). This member is usually optional, and not used by the majority of the algorithms in PCL.

## sensor_orientation_ (`Eigen::Quaternionf`)

Specifies the sensor acquisition pose (**orientation**). This member is usually optional, and not used by the majority of the algorithms in PCL.

To simplify development, this class also provides a number of helper member functions. The **PointT** type is the primary point data type and describes what each individual element of points holds. PCL comes with a large variety of different point types

