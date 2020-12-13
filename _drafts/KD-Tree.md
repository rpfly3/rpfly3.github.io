# KD-Tree

In this section, we will go over how to use a KD-Tree for

1. Finding the K nearest neighbors of a specific point or location.
2. Finding all neighbors within some radius specified by the user.

## Theoretical primer

> A KD-Tree, or k-dimensional tree, is a data structure used in computer science for organizing some number of points in a space with k dimensions.

Actually KD-Tree is a **binary search tree**, however with other constraints imposed on it. It is mainly used for 

- range search
- nearest neighbor search

For our purposes, we will generally only be dealing with point clouds in three dimensions, so all of our KD-trees will be **three-dimensional**. 

Each level of a KD-Tree splits all children along a specific dimension, using a hyperplane that is perpendicular to the corresponding axis. 

- At the root of the tree all children will be split based on the first dimension (i.e. if the first dimension coordinate is less than the root it will be in the left-sub tree and if it is greater than the root it will obviously be in the right sub-tree).
- Each level down in the tree divides on the next dimension, returning to the first dimension once all others have been exhausted. 

The most efficient way to build a KD-Tree is to use a partition method like the one Quick Sort uses to place the median point at the root and everything with a smaller one-dimensional value to the left and larger to the right. You then repeat this procedure on both the left and right sub-trees until the last trees that you are to partition are only composed of one element. 