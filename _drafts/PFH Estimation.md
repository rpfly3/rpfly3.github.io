# Point Feature Histograms (PFH) descriptors

As point feature representations go, surface normals and curvature estimates are somewhat basic in their representations of the geometry around a specific point. Though extremely fast and easy to compute, they cannot capture too much detail, as they approximate the geometry of a point's k-neighborhood with only a few values. As a direct consequence, most scenes will contain many points with the same or very similar feature values, thus reducing their informative characteristics.

This tutorial introduces a family of 3D feature descriptors coined PFH (Point Feature Histograms) for simplicity, presents their theoretical advantages and discusses implementation details from PCL's perspective.

## Theoretical primer

> The goal of the PFH formulation is to encode a point's k-neighborhood geometrical properties by generalizing the mean curvature around the point using a multi-dimensional histogram of values. 

This highly dimensional hyperspace provides an informative signature for the feature representation, is invariant to the 6D pose of the underlying surface, and copes very well with different sampling densities or noise levels present in the neighborhood.

A Point Feature Histogram representation is based on the relationships between the points in the k-neighborhood and their estimated surface normals. Simply put,

> it attempts to capture as best as possible the sampled surface variations by taking into account all the interactions between the directions of the estimated normals. **The resultant hyperspace is thus dependent on the quality of the surface normal estimations at each point**.

The final PFH descriptor is computed as a histogram of relationships between all pairs of points in the neighborhood, and thus has a computational complexity of $O(k^2)$. 

To compute the relative difference between two points $p_i$ and $p_j$ and their associated normals $n_i$ and $n_j$, we define a fixed coordinate frame at one of the points.

$${\mathsf u} = \boldsymbol{n}_s \\ {\mathsf v} = {\mathsf u} \times \frac{(\boldsymbol{p}_t-\boldsymbol{p}_s)}{{\|\boldsymbol{p}_t-\boldsymbol{p}_s\|}_{2}} \\ {\mathsf w} = {\mathsf u} \times {\mathsf v}$$

Using the above **uvw** frame, the difference between the two normals $n_s$ and $n_t$ can be expressed as a set of angular features as follows:

$$\alpha = {\mathsf v} \cdot \boldsymbol{n}_t \\ \phi = {\mathsf u} \cdot \frac{(\boldsymbol{p}_t - \boldsymbol{p}_s)}{d}\\ \theta = \arctan ({\mathsf w} \cdot \boldsymbol{n}_t, {\mathsf u} \cdot \boldsymbol{n}_t) \\$$

where **d** is the Euclidean distance between the two points $\boldsymbol{p}_s$ and $\boldsymbol{p}_t$, $d={\|\boldsymbol{p}_t-\boldsymbol{p}_s\|}_2$. The quadruplet $\langle\alpha, \phi, \theta, d\rangle$ is computed for each pair of two points in k-neighborhood, therefore reducing the 12 values (xyz and normal information) of the two points and their normals to 4.

To estimate a PFH quadruplet for a pair of points, use:

```c++
computePairFeatures(const Eigen::Vector4f& p1, const Eigne::Vector4f& n1, const Eigen::Vector4f& p2, const Eigne::Vector4f& n2, float& f1, float& f2, float& f3, float& f4);
```

To create the final PFH representation for the query point, the set of all quadruplets is binned into a histogram. The binning process divides each feature's value range into **b** subdivisions, and counts the number of occurrences in each subinterval. **Since three out of the four features presented above are measure of the angles between normals, their values can easily be normalized to the same interval on the trigonometric circle.**  

A binning example is to divide each feature interval into the same number of equal parts, and therefore create a histogram with b^4 bins in a fully correlated space. In this space, a histogram bin increment corresponds to a point having certain values for all its 4 features.

> In some cases, the fourth feature, **d**, does not present an extreme significance for 2.5D datasets, usually acquired in robotics, as the distance between neighboring points increases from the viewpoint. Therefore, omitting **d** for scans where the local point density influences this feature dimension has proved to be beneficial.

## Estimating PFH features

The default PFH implementation uses 5 binning subdivisions (e.g., each of the four feature values will use this many bins from its value interval), and does not include the distances (as explained above - although the `computePairFeatures` method can be called by the user to obtain the distances too, if desired) which results in a 125-byte array ($3^5$) of float values. These are stored in a `pcl::PFHSignature125` point type.

The actual `compute` call from the `PFHEstimation` class does nothing internally but:

```pseudocode
for each point p in cloud P
	1. get the nearest neighbors of p
	2. for each pair of neighbors, compute the three angular values
	3. bin all the results in an output histogram
```

To compute a PFH representation from a k-neighborhood, use:

```c++
computePointPFHSignature(const pcl::PointCloud<PointT>& cloud, const pcl::PointCloud<PointT>& normals, const std::vector<int>& indices, int nr_split, Eigen::VectorXf& pfh_histogram);
```

where `cloud` is the input point cloud that contains the points, `normals` is the point cloud that contains the normals, `indices` represents the set of k-nearest neighbors from `cloud`, `nr_split` is the number of subdivisions to use for the binning process for each feature interval, and `pfh_histogram` is the output resultant as an array of float values.

