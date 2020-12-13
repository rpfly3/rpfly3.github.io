## Estimating Surface Normals in a PointCloud

Given a geometric surface, it's usually trivial to infer the direction of the normal at a certain point on the surface as the vector perpendicular to the surface in that point. However, since the point cloud datasets that we acquire represent a set of point samples on the real surface, there are two possibilities:

- Obtain the underlying surface from the point cloud dataset, using surface meshing techniques, and then compute the surface normals from the mesh
- Use approximations to infer the surface normals from the point cloud directly.

This tutorial will address the latter, that is, given a point cloud dataset, directly compute the surface normals at each point in the cloud.

## Theoretical Primer

> The problem of determining the normal to a point on the surface is approximated by the problem of estimating the normal of a plane tangent to the surface, which in turn becomes a least-square plane fitting estimation problem.

The solution for estimating the surface normal is therefore reduced to an analysis of the eigenvectors and eigenvalues (or PCA – Principal Component Analysis) of a covariance matrix created from the nearest neighbors of the query point. More specifically, for each point $\boldsymbol{p}_i$ , we assemble the covariance matrix $\mathcal{C}$ as follows:
$$\mathcal{C} = \frac{1}{k}\sum_{i=1}^{k}{\cdot (\boldsymbol{p}_i-\overline{\boldsymbol{p}})\cdot(\boldsymbol{p}_i-\overline{\boldsymbol{p}})^{T}}, ~\mathcal{C} \cdot \vec{{\mathsf v}_j} = \lambda_j \cdot \vec{{\mathsf v}_j},~ j \in \{0, 1, 2\}$$

Where $k$ is the number of point neighbors considered in the neighborhood of $\boldsymbol{p}_i$, $\overline{\boldsymbol{p}}$ represents the 3D centroid of the nearest neighbors, $\lambda_j$ is the $j$-th eigenvalue of the covariance matrix, and $\vec{{\mathsf v}_j}$ the $j$-th eigenvector.

To estimate a covariance matrix from a set of points in PCL, you can use:

```c++
// Placeholder for the 3 x 3 covariance matrix at each surface patch
Eigen::Matrix3f covariance_matrix;
// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
Eigen::Vector4f xyz_centroid;
// Estimate the XYZ centroid
compute3DCentroid(cloud, xyz_centroid);
// Compute the 3 x 3 covariance matrix
computeCovarianceMatrix(cloud, xyz_centroid, covariance_matrix);
```

In general, because there is no mathematical way to solve for the sign of the normal, its orientation computed via Principal Component Analysis (PCA) as shown above is ambiguous, and not consistently oriented over an entire point cloud dataset. The solution to this problem is trivial if the viewpoint ${\mathsf v}_p$ is in fact known. To orient all normals $\vec{\boldsymbol{n}}_i$ consistently towards the viewpoint, they need to satisfy the equation: $$\vec{\boldsymbol{n}}_i \cdot ({\mathsf v}_p - \boldsymbol{p}_i) > 0$$

To re-orient a given point normal manually in PCL, you can use

```c++
flipNormalTowardsViewpoint(const PointT& point, float vp_x, float vp_y, float vp_z, Eigen::Vector4f& normal);
```

## Selecting the right scale

The specifics of the nearest-neighbor estimation problem raise the question of the *right scale factor*:

> Given a sampled point cloud dataset, what are the correct **k** (given via `pcl::Feature::setKSearch`) or **r** (given via `pcl::Feature::setRadiusSearch`) values that should be used in determining the set of nearest neighbors of a point ?

Without going into too many details, it suffices to assume that for now, the scale for the determination of a point's neighborhood has to be selected **based on the level of detail required by the application**. Simply put, if the curvature at the edge between the handle of a mug and the cylindrical part is important, the scale factor needs to be small enough to capture those details, and large otherwise.

## Estimating the normals

The actual `compute` call from the `NormalEstimation` class does nothing internally but:

```pseudocode
for each point p in cloud P
	1. get the nearest neighbors of p
	2. compute the surface normal n of p
	3. check if n is consistently oriented towards the viewpoint and flip otherwise
```

The viewpoint is by default `(0, 0, 0)` and can be changed with:

```c++
setViewPoint(float vpx, float vpy, float vpz);
```

To compute a single point normal, use:

```c++
computePointNormal(const pcl::PointCloud<PointT>& cloud, const std::vector<int>& indices, Eigen::Vector4f& plane_parameters, float& curvature);
```

Where *cloud* is the input point cloud that contains the points, `indices` represents the set of k-nearest neighbors from *cloud*, and plane_parameters and curvature represent the output of the normal estimation, with *plane_parameters* holding the normal `(nx, ny, nz)` on the first 3 coordinates, and the fourth coordinate is `D = nc . p_plane (centroid here) + p`. The output surface curvature is estimated as a relationship between the eigenvalues of the covariance matrix (as presented above), as: $$\sigma = \frac{\lambda_0}{\lambda_0 + \lambda_1 + \lambda_2}$$.

## Speeding Normal Estimation with OpenMP

For the speed-savvy users, PCL provides an additional implementation of surface normal estimation which uses multi-core/multi-threaded paradigms using OpenMP to speed the computation. The name of the class is `pcl::NormaleEstimationOMP`, and its API is 100% compatible to the single-threaded `pcl::NormalEstimation`, which makes it suitable as a drop-in replacement. On a system with 8 cores, you should get anything between 6-8 times faster computation times.

