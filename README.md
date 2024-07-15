# Point Cloud Plane Fitting

In this project I focused on fitting planes to a 3D point cloud using RANSAC (Random Sample Consensus) and visualizing the 
fitted planes. The code can also detect multiple planes in the point cloud, and it allows the user to visualize them 
using Open3D.

## Features

- Generate a simple point cloud from a depth map.
- Detect and visualize a *single* plane in the point cloud.
- Detect and visualize *multiple* planes in the point cloud.
- Calculate mean, standard deviation for the points relative to the fitted planes.
- User interface for selecting which planes to visualize.


![flat wall fitted with pointcloud](pictures%2FScreenshot%20from%202024-07-12%2018-15-47.png)
![flat wall fitted with pointcloud](pictures%2FScreenshot%20from%202024-07-12%2018-16-25.png)

## Plane mesh using Open3D
The plane mesh is done using an Open3D function `TriangleMesh`. The plane is essentially represented by a triangle mesh 
that contains only two triangles. 

First step is to calculate the vertices of the plane (as we want the plane to be visualised as a finite rectangle).
Then we create two triangles and assign both to a TriangleMesh object in Open3D.

## RANSAC 
RanSaC (Random Sample Consensus) is an iterative method for estimating parameters of a mathematical model from a data that contains outliers.

### Steps of RANSAC
    For each iteration:
        Random Sampling:
            Randomly select a minimal subset of data points required to estimate the model parameters.
        Model Estimation:
            Compute the model parameters using the selected subset.
        Inlier Identification:
            Determine how many of the remaining data points fit the estimated model within the predefined threshold.
        Model Evaluation:
            If the estimated model produces more inliers than the current best model, update the best model to the current model.

    Result:
        After the maximum number of iterations, the model with the highest number of inliers is selected as the best model.

### Advantages of RANSAC
- Can handle a significant proportion of outliers in the data.

### Disadvantages of RANSAC
- The number of iterations needed can be high, especially with a large number of outliers.
- Parameter Sensitivity

### Plane detection
Each plane is detected using RANSAC method. For detecting multiple planes, RANSAC is applied several times. After each iteration,
The points which are considered inliers are removed from the point cloud and then RANSAC is repeated on the new smaller point cloud.


## Why is detecting planes in point clouds exciting?
[Detection of walls, floors and ceilings](https://core.ac.uk/download/pdf/35279765.pdf)
![Boxes on the ground](pictures%2FScreenshot%20from%202024-07-12%2018-38-23.png)
or for example [Automated control of walls on highways](https://itc.scix.net/pdfs/w78-2014-paper-120.pdf)

## Sources and interesting readings
- Motivation for this problem: I found [this article](https://www.mathworks.com/help/vision/ref/pcfitplane.html) while
struggling with meshing and got inspired to implement it slightly differently myself. In terms of accuracy, my approach is 
less accurate, but I don't need to input normals of the planes. Which means my approach requires less 
prior knowledge about the data.
- [Tutorial on how to use Tkinter](https://www.geeksforgeeks.org/python-tkinter-tutorial/)
- More about Open3D can be found in [documentation](https://www.open3d.org/docs/0.9.0/tutorial/Basic/index.html)
- The beginning of [this article](https://medium.com/@mustafaboyuk24/obtaining-point-cloud-from-depth-images-with-intel-realsense-d-435-camera-144e8ef9260d)
explains how coordinates are transformed when creating a point cloud from a depth map.