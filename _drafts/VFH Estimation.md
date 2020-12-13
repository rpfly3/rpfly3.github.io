# Estimating VFH signatures for a set of points

This tutorial describes the **Viewpoint Feature Histogram (VFH)** descriptor, a novel representation for point clusters for the problem of Cluster (e.g., Object) Recognition and 6-DOF Pose Estimation.

## Theoretical primer

The Viewpoint Feature Histogram (VFH) has its roots in the FPFH descriptor. Due to its speed and discriminative power, we decided to leverage the strong recognition results of FPFH, but to add in **viewpoint variance** while retaining invariance to scale. 

The contribution of VFH to the problem of object recognition and pose identification was 

- to extend the FPFH to estimated for the entire object cluster, 
- and to compute additional statistics between the viewpoint direction and the normals estimated at each point. 

To do this, we use the key idea of mixing the viewpoint direction directly into the relative normal angle calculation in the FPFH.

> The viewpoint component is computed by collecting a histogram of the angles that the viewpoint direction makes with each normal. Note, we do not mean the view angle to each normal as this would not be scale invariant, but instead we mean the angle between the central viewpoint direction translated to each normal.
>
> The second component measures the relative span, tilt and yaw angles as described in FPFH descriptors, but now measured between the viewpoint direction at the central point and each of the normals on the surface.

The new assembled feature is therefore called the Viewpoint Feature Histogram (VFH). It has two parts:

- A viewpoint direction component
- A surface shape component comprised of an extended FPFH

## Estimating VFH features

The default VFH implementation uses 45 binning subdivisions for each of the tree extended FPFH values, plus another 45 binning subdivisions for the distances between each point and the centroid and 128 binning subdivisions for the viewpoint component, which results a 308-byte array of float values. These are stored in a `pcl::VFHSignature308` point type.

The major difference between the PFH/FPFH descriptors and VFH, is that for a given point cloud dataset, only a single VFH descriptor will be estimated, while the resultant PFH/FPFH data will have the same number of entries as the number of points in the cloud.

## Visualizaing VFH signatures

`libpcl_visualization` contains a special `PCLHistogramVisualization` class, which is also used by `pcd_viewer` to automatically display the VFH descriptors as a histogram of float values.