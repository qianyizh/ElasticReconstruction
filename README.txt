Code under maintenance, please do NOT download it yet.

===============================================================================
ElasticReconstruction: dense scene reconstruction with elastic fragments.

===============================================================================
Introduction

This is an open source C++ implementation based on the technique presented in
the following papers:

Dense Scene Reconstruction with Points of Interest, SIGGRAPH 2013
Qian-Yi Zhou and Vladlen Koltun

Elastic Fragments for Dense Scene Reconstruction, ICCV 2013
Qian-Yi Zhou, Stephen Miller and Vladlen Koltun

Project page:
http://www.stanford.edu/~qianyizh/projects/elasticreconstruction.html
http://www.stanford.edu/~qianyizh/projects/scene.html
Data page:
http://www.stanford.edu/~qianyizh/projects/scenedata.html

This github repository is maintained by Qian-Yi Zhou (Qianyi.Zhou@gmail.com)
Contact me or Vladlen Koltun (vkoltun@gmail.com) if you have any questions.

===============================================================================
License

The source code is released under GPL license.

We encourage you to use the code freely for research purposes.
Please cite our ICCV 2013 paper "Elastic Fragments for Dense Scene
Reconstruction" if you use our code or data.

For other purposes, please contact us.
Stanford OTL is our interface for commercial use of the code, details refer to
http://techfinder.stanford.edu/technology_detail.php?ID=30418

The data on our data page can be used for any purposes with proper attribution.
I scanned the scenes and reconstructed them. Just use them if you like.

===============================================================================
Modules

+ FragmentOptimizer
This is the main function of this package. The purpose is to estimate an 
optimal nonrigid deformation for each fragment (individual point clouds) so
as to minimize the alignment error. It takes fragments, dense correspondences, 
and an initial camera pose trajectory as input, and outputs control lattices
for the fragments. See its readme for more information.

+ BuildFragment
This module calls KinectFusion in PCL and creates short-term fragments from
the raw RGB-D stream. Ralative poses from depth frames to the corresponding
fragment are also recorded and stored in a .log file.

+ BuildCorrespondence
This is a pre-processing step of FragmentOptimizer. It takes fragments and an
initial camera pose trajectory as input, applies ICP registration and outputs
dense correspondences between aligned fragment pairs.

+ DepthReproject
This program applies the nonrigid transformation onto the original RGB-D 
stream. It takes the original RGB-D stream, control lattices, an initial 
trajctory, and relative camera poses (that maps each frame to the fragment) as
input, and outputs a deformed RGB-D stream.

+ Integrate

===============================================================================
Dependencies

We strongly recommend you install Point Cloud Library (PCL) x64 for Windows.
http://pointclouds.org/

Although the main module (FragmentOptimizer) can be de-PCL-ed easily, it would
save you a lot of trouble if you have experiences with PCL. Other modules are 
all heavily relying on PCL.

FragmentOptimizer uses SuiteSparse and a different version of Eigen. They both
are included in the "external" directory under the project.

===============================================================================
Quick Start

Download the data package from:

Then

===============================================================================
Compilation

We include project files for MSVS2010.

==============================================================================
