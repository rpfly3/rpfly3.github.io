# How 3D Features work in PCL

Applications which need to compare points for various reasons require better characteristics and metrics to be able to distinguish between geometric surfaces. The concept of a 3D point as a singular entity with Cartesian coordinates therefore disappears, and a new concept, that of **local descriptor** takes its place. The literature is abundant of different naming schemes describing the same conceptualization, such as **shape descriptor** or **geometric features** but they will be referred to as **point feature representations**.

By including the surrounding neighbors, the underlying sampled surface geometry can be inferred and captured in the feature formulation, which contributes to solving the ambiguity comparison problem. Ideally, the resultant features would be very similar (with respect to some metric) for points residing on the same or similar surfaces, and different for points found on different surfaces. A **good** point feature representation distinguishes itself from a **bad** one, by being able to capture the same local surface characteristics in the presence of :

- Rigid transformations - that is, 3D rotations and 3D translations in the data should not influence the resultant feature vector F estimation.
- Varying Sampling density - in principle, a local surface patch sampled more or less densely should have the same feature vector signature.
- Noise - the point feature representation must retain the same or very similar values in its feature vector in the presence of mild noise in the data.

In general, PCL features use approximate methods to compute the nearest neighbors of a query point, using fast KD-Tree queries. There are two types of queries that we're interested in:

- Determine the **K** (user given parameter) neighbors of a query point (also known as **K-Search**).
- Determine **all the neighbors** of a query point within a sphere of radius **r**(also known as **Radius-Search**).

## How to pass the input

As almost all classes in PCL that inherit from the base `pcl::PCLBase` class, the `pcl::Feature` class accepts input data in two different ways:

1. **Mandatory** - an entire point cloud dataset, given via `setInputCloud(PointCloudConstPtr &)`: any feature estimation class with attempt to estimate a feature at **every** point in the given input cloud.
2. **Optional** - a subset of a point cloud dataset, given via `setInputCloud(PointCloudConstPtr &)` and `setIndices(IndicesConstPtr &)`: any feature estimation class will attempt to estimate a feature at every point in the given input cloud that has an index in the given indexes list. By default, if no set of indexes is given, all points in the cloud will be considered.

In addition, the set of point neighbors to be used, can be specified through an additional call, `setSearchSurface(PointCloudConstPtr &)`. This call is optional, and when the search surface is not given, the input point cloud dataset is used instead.

The most useful example when `setSearchSurface()` should be used, is when we have a very dense input dataset, but we do not want to estimate features at all the points in it, but rather at some keypoints discovered using the methods in `pcl_keypoints`, or at a downsampled version of the cloud (e.g., obtained using a `pcl::VoxelGrid<T>` filter). In this case, we pass the downsampled/keypoints input via `setInputCloud()`, and the original data as `setSearchSurface()`. 

## An example for normal estimation

Once determined, the neighboring points of a query point can be used to estimate a local feature representation that captures the geometry of the underlying sampled surface around the query point. 

An important problem in describing the geometry of the surface is to first infer its orientation in a coordinate system, that is, estimate its normal. Surface normals are important properties of a surface and are heavily used in many areas such as computer graphics applications to apply the correct light sources that generate shadings and other visual effects. 

