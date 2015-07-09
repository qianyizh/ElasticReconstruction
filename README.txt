===============================================================================
=                       Robust Scene Reconstruction                           =
===============================================================================

LATEST NEWS (7/9/2015):

1. Executable system available at http://redwood-data.org/indoor/tutorial.html

2. Latest algorithms in our CVPR 2015 paper "Robust Reconstruction of Indoor
Scenes" added to the package.

3. Lots of useful things - data, evaluation tools, etc - on our new project
website: http://redwood-data.org/indoor/

===============================================================================

Introduction

This is an open source C++ implementation based on the technique presented in
the following papers:

Robust Reconstruction of Indoor Scenes, CVPR 2015
Sungjoon Choi, Qian-Yi Zhou, and Vladlen Koltun

Simultaneous Localization and Calibration: Self-Calibration of Consumer Depth 
Cameras, CVPR 2014
Qian-Yi Zhou and Vladlen Koltun

Elastic Fragments for Dense Scene Reconstruction, ICCV 2013
Qian-Yi Zhou, Stephen Miller and Vladlen Koltun

Dense Scene Reconstruction with Points of Interest, SIGGRAPH 2013
Qian-Yi Zhou and Vladlen Koltun

Main project page:
http://redwood-data.org/indoor/

This github repository is maintained by Qian-Yi Zhou (Qianyi.Zhou@gmail.com)
Contact me or Vladlen Koltun (vkoltun@gmail.com) if you have any questions.

===============================================================================

License

The source code is released under MIT license.

In general, you can do anything with the code for any purposes, with proper 
attribution. If you do something interesting with the code, we'll be happy to 
know about it. Feel free to contact us.

For more license information including citation instructions, refer to:
http://redwood-data.org/indoor/pipeline.html

===============================================================================

Modules

+ GlobalRegistration
A state-of-the-art global registration algorithm that aligns point clouds
together.

+ GraphOptimizer
Pose graph optimization that prunes false global registration results. See CVPR
2015 paper for details.

+ FragmentOptimizer
The core function that simultaneously optimizes point cloud poses and a
nonrigid correction pattern. See CVPR 2014 and ICCV 2013 papers for details.

+ BuildCorrespondence
ICP refinement for point cloud pairs registered by GlobalRegistration module.

+ Integrate
A CPU-based algorithm that integrates depth images into a voxel, based on
camera pose trajectory and nonrigid correction produced by previous steps.

+ Matlab_Toolbox
A Matlab toobox for evaluation of camera pose trajectory and global
registration.

+ In the executable package
    * pcl_kinfu_largeScale_release.exe
    * pcl_kinfu_largeScale_mesh_output_release
Executable files for creating intermediate point clouds and final mesh.

===============================================================================

Quick Start

See tutorial on this page:
http://redwood-data.org/indoor/tutorial.html

===============================================================================

Build Dependencies

We strongly recommend you *compile* Point Cloud Library (PCL) x64 with
Visual Studio. http://pointclouds.org/

SuiteSparse is required for solving large sparse matrices.
https://github.com/PetterS/CXSparse

ACML is required for SuiteSparse.
http://developer.amd.com/tools-and-sdks/cpu-development/amd-core-math-library-acml/

The compilation requires Visual Studio 2010 on a Windows 7/8.1 64bit system.

We are not happy with the current compatibility issues. We are working on a new
code release that will not depend on external libraries as much and will be
much easier to compile. Stay tuned.
