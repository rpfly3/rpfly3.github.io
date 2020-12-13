## Point Cloud Compression

Point clouds consist of huge data sets describing three dimensional points associated with additional information such as distance, color, normals, etc. Additionally, they can be created at high rate and therefore occupy a significant amount of memory resources. Once point clouds have to be stored or transmitted over rate-limited communication channels, methods for compressing this kind of data become highly interesting. The Point Cloud Library provides point cloud compression functionality. It allows for encoding all kinds of point clouds including "unorganized" point clouds that are characterized by non-existing point references, varying point size, resolution, density and/or point ordering. Furthermore, the underlying octree data structure enables to efficiently merge point cloud data from several sources. 

## Spatial change detection on unorganized point cloud data

An octree is a tree-based data structure for organizing sparse 3-D data. In this tutorial we will learn how to use the octree implementation for detecting spatial changes between multiple unorganized point clouds which could vary in size, resolution, density and point ordering. 

> By recursively comparing the tree structures of octrees, spatial changes represented by differences in voxel configuration can be identified. 

Additionally, the pcl octree "double buffering" technique allows us to efficiently process multiple point clouds over time.