# 3D Point Cloud

## Introduction

### Advantages & Difficulties

#### Difficulties of Point Cloud Processing

- Sparsity
- Irregular - difficulty in neighbor searching
- Lack of texture information
- Un-ordered - difficulty in deep learning
- Rotation equivalence / Invariance

### Course Outline

#### Classical Methods

- Pros
  - Explainable - it follows physics and we know why it works/doesn't work
  - Controllable - We know how to debug
- Cons
  - Hard to model semantics
  - User-unfriendly

#### Deep Learning Methods

- Pros

  - Simple
  - High performance
  - Data driven

- Cons

  - Un-explainable - No one knows why/how
  - Un-controllable - Black box
  - Requires special hardware - GPU/FPGA, etc.
  - Simple - The barrier is lower and lower means it will be more and more difficult to find a job.

  

## Principle Component Analysis

> PCA is to find the dominant directions of the point cloud.

#### Applications

- Dimensionality reduction
- Surface normal estimation
- Canonical orientation
- Keypoint detection
- Feature description



### Surface Normal

#### Surface normal on surface

> The vector perpendicular to the tangent plane of the surface at a point P.

#### Applications

- Segmentation / Clustering
- Plane detection
- Point cloud feature for applications like Deep Learning

### Surface Normal - How to compute

#### Surface normal on 3D point cloud

1. Select a point P
2. Find the neighborhood that defines the surface
3. PCA
4. Normal -> the least significant vector
5. Curvature -> ratio between eigen values $\lambda_3/(\lambda_1 + \lambda_2 + \lambda_3)$. 

### Filters

#### Noise removal

- Radius Outlier Removal
- Statistical Outlier Removal

#### Downsampling

- Voxel Grid Downsampling
  - Exact / Approcimated
  - Centroid / Random Selection
- Farthest Point Sampling
- Normal Space Sampling

#### Upsampling / Smoothing / Noise Removal

- Bilateral Filter

## Nearest Neighbors

### Why NN is difficult for point clouds

#### For point clouds

- Irregular - no grid based representation
- Curse of dimensionality
  - Non-trivial to build grids
  - Non-trivial to sort or build spatial partitions
- Huge data throughput in real-time applications