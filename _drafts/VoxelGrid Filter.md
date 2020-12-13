## Downsampling a PointCloud using a `VoxelGrid` filter

The `VoxelGrid` class that we're about to present creates a 3D voxel grid (think about a voxel grid as a set of tiny 3D boxes in space) over the input point cloud data. Then, in each voxel, all the points present will be approximated (i.e., downsampled) with their centroid. This approach is a bit slower than approximating them with the center of the voxel, but it represents the underlying surface more accurately.

## Removing outliers using a `StatisticalOutlierRemoval` filter

In this tutorial we will learn how to remove noisy measurements, e.g. outliers, from a point cloud dataset using statistical analysis techniques.

Laser scans typically generate point cloud datasets of varying point densities. Additionally, measurement errors lead to sparse outliers which corrupt the results even more. This complicates the estimation of local point cloud characteristics such as surface normals or curvature changes, leading to erroneous values, which in turn might cause point cloud registration failures. 

Some of these irregularities can be solved by performing a statistical analysis on each point's neighborhood, and trimming those which do not meet a certain criteria. Our sparse outliers removal is based on the computation of the distribution of point to neighbors distances in the input dataset. For each point, we compute the mean distance from it to all its neighbors. By assuming that the resulted distribution is **Gaussian** with a mean and standard deviation, all points whose mean distances are outside an interval defined by the **global distances mean** and **standard deviation** can be considered as outliers and trimmed from the dataset.

