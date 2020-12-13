# Benchmarking Feature Descriptor Algorithms

In this tutorial, we will go over how to use the `FeatureEvaluationFramework` class to benchmark various feature descriptor algorithms. The benchmarking framework allows the testing of different kinds of feature descriptor algorithms, over a choice of independent variables, ranging from *input clouds*, *algorithm parameters*, *downsampling leaf size*, and *search threshold*.

## Using the `FeatureCorrespondenceTest` Class

The `FeatureCorrespondenceTest` class performs a single "feature correspondence test" as follows:

1. The `FeatureCorrespondenceTest` class takes two input clouds (source and target).

2. It will compute feature descriptors in each cloud using the specified algorithm and parameters.

3. Each feature in the source cloud will be matched to its corresponding feature in the target cloud based on a nearest neighbor search in the n-D feature space.
4. For each point, the system will compare the 3D position of the estimated corresponding target point to the previously estimated ground truth position. 
5. If the two points are close together (as determined by a user specified threshold), then the correspondence is marked as a success. Otherwise, it is marked a failure. 
6. The total number of successes and failures will be calculated and stored for further analysis.

The functions which perform common tasks are provided in the `FeatureCorrespondenceTest` class. Functions specific to the Feature Descriptor algorithm (i.e. the actual computation of features) must be defined in a separate child class derived from `FeatureCorrespondenceTest`.

## Using the `FeatureEvaluationFramework` Class

The `FeatureEvaluationFramework` class encapsulates the actual benchmarking functionality. It has one template parameter, which should match the `point_type` of the point clouds to be given as input to the framework.

To initialize the `FeatureEvaluationFramework` object, the following set of functions should be called:

- `setFeatureTest` : Choose the Feature Descriptor algorithm to be tested

- `setInputClouds` : Load source and target clouds from `.pcd` files

- `setGroundTruth` : Load the ground truth transformation as 4 x 4 matrix, from a file

- `setThreshold` : Either a single threshold value, or a threshold range, specified by lower bound, upper bound, and delta

- `setParameters` : Specific to the Feature Descriptor algorithm, to be given as a “key1=value1, key2=value2, ...” string

- `setDownsampling` : Choose whether to filter input clouds through **VoxelGrid filter**

- `setLeafSize` : Set VoxelGrid leaf size for downsampling input clouds

- `setLogfile` : Set log file to store output of the test in CSV format

- `setVerbose`: Choose whether to show step by step progress of the benchmarking on the console

After this, we are ready to perform the test. There are two possibilities:

- Single Test : Perform a single feature extraction and store the output statistics by calling `runSingleTest()`.

- Multiple Tests : Perform multiple tests with varying a single independent variable, and store the set of outputs in CSV format (in the log file).

  The supported function calls are:

  - `runMultipleFeatures()` - Perform test with multiple feature descriptor algorithms
  - `runMultipleClouds()` - Run same feature evaluation on a (large) set of input clouds
  - `runMultipleParameters()` - Perform the feature evaluation with varying parameter values
  - `runMultipleLeafSizes()` - Vary the VoxelGrid leaf size for downsampling the input cloud before performing each evaluation

  The values for the independent variable are taken as input from a text file, which should be provided as an argument to this function. For example, `runMultipleLeafSizes(std::string filename)` will read values for the VoxelGrid leaf size from each line of “filename”, and perform a feature extraction for each leaf size, storing the set of results in the output log file.

## Supported Datasets And Ground-truth Format

The input clouds (source and target) are read from `.pcd` files. The point type of the clouds should match the template parameter used in `FeatureEvaluationFramework`, i.e. to use `PointXYZRGB` clouds, create an object of type `FeatureEvaluationFramework<PointXYZRGB>` to perform the evaluation.

The ground truth is a `(Vector3f, Quaternionf)` pair corresponding to the rigid transformation mapping the source and target clouds. Currently, the ground truth matrix is read from a file containing 7 values on one line, where first three values are taken as a `Vector3f` and next four are taken as a `Quaternionf`.

The conference room dataset contains 350 point clouds (`cloud_###.pcd`) and corresponding 7-D pose of the camera (`pose_###.txt`). We will be using clouds from this dataset to perform the benchmarking, however the framework can easily incorporate other datasets in the future.

## Adding New Features To The Framework

Adding new feature algorithms to the testing framework is very straightforward: one needs to derive a new child from the `FeatureCorrespondenceTest` base class, and implement the following member functions:

- `setParameters (ParameterList params)`

  ​	Here `ParameterList` is a map<string, string> which contains (key,value) pairs of the different parameters passed to the algorithm. This function should parse the provided `ParameterList` and store the relevant parameter values, to be used in feature computation.

- `computeFeatures (double& time_source, double& time_target)`

  ​	This function should perform the actual computation of source and target features, and the implementation details will vary according to the algorithm in question. The arguments time_source, and time_target should be filled with the time taken for source and target feature computation respectively.

- `computeFeatures ()`

  ​	Same as above, except runtime statistics are not to be measured.

- `computeCorrespondences ()`

  ​	For each source point, compute its nearest neighbor from the target cloud, in the n-D feature space, and store the correspondences in `MapSourceToTargetIndices`.

Please look at the `FPFHTest` implementation for reference (in /trunk/features/include/pcl/features/feature_evaluation)

