# Fast Point Feature Histograms (PFH) descriptors

The theoretical computational complexity of the Point Feature Histogram for a given point cloud $P$ with $n$ points is $O(nk^2)$, where $k$ is the number of neighbors for each point $p$ in $P$. For real-time or near real-time applications, the computation of Point Feature Histograms in dense point neighborhoods can represent one of the major bottlenecks.

This tutorial describes a simplification of the PFH formulation, called **Fast Point Feature Histograms (FPFH)**, that reduces the computational complexity of the algorithm to $O(nk)$, while still retaining most of the discriminative power of the PFH.

## Theoretical primer

To simplify the histogram feature computation, we proceed as follows:

1. In the first step, for each query point $p_q$ a set of tuples $\alpha, \phi, \theta$ between itself and its neighbors are computed as described in Point Feature Histograms (PFH) descriptors. This is called the **Simplified Point Feature Histogram (SPFH)**.
2. In the second step, for each point its $k$ neighbors are re-determined, and the neighboring SPFH values are used to weight the final histogram of $p_q$ (called FPFH) as follows: $$FPFH(\boldsymbol{p}_q) = SPFH(\boldsymbol{p}_q) + {1 \over k} \sum_{i=1}^k {{1 \over \omega_k} \cdot SPFH(\boldsymbol{p}_k)}$$

where the weight $\omega_k$ represents a distance between the query point $p_q$ and a neighbor point $p_k$ in some given metric space, thus scoring the $(p_q, p_k)$ pair, but could just as well be selected as a different measure if necessary.

## Differences between PFH and FPFH

The main differences between the PFH and FPFH formulations are summarized below:

1. the FPFH does not fully interconnect all neighbors of $p_q$, and is thus missing some value pairs which might contribute to capture the geometry around the query point.
2. the PFH models a precisely determined surface around the query point, while the FPFH includes additional point pairs outside the the $r$ radius sphere (though at most $2r$ away).
3. because of the re-weighting scheme, the FPFH combines SPFH values and recaptures some of the point neighboring value pairs.
4. the overall complexity of FPFH is greatly reduced, thus making possible to use it in real-time applications.
5. the resultant histogram is simplified by decorrelating the values, that is simply creating $d$ separate feature histograms, one for each feature dimension, and concatenate them together. 

## Estimating FPFH features

The default FPFH implementation uses 11 binning subdivisions (e.g., each of the four feature values will use this many bins from its value interval), and a decorrelated scheme (the feature histograms are computed separately and concatenated) which results in a 33-byte array of float values. These are stored in a `pcl::FPFHSignature33` point type.

The actual `compute` call from the `FPFHEstimation` class does nothing internally but

```pseudocode
for each point p in cloud P
	1. pass 1:
		- get the nearest neighbors of p
		- for each pair of (p, $p_k$) (where p_k is a neighbor of p), compute the three angular values
		- bin all the results in an output SPFH histogram
	2. pass 2:
		- get the nearest neighbors of p
		- use each SPFH of p_k with a weighting scheme to assemble the FPFH of p
```



## Speeding FPFH with OpenMP

For the speed-savy suers, PCL provides an additional implementation of FPFH estimation which uses multi-core/multi-threaded paradigms using OpenMP to speed the computation. The name of the class is `pcl::FPFHEstimationOMP`, and its API is 100% compatible to the single-threaded `pcl::FPFHEstimation`, which makes it suitable as a drop-in replacement. On a system with 8 cores, you should get anything between 6-8 times faster computation times.