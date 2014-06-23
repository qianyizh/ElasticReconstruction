LATEST NEWS:

1. The code is ready to use.
I am going to update the document and the dataset soon.

2. The code is now under MIT license.
See the License section for details.

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

Simultaneous Localization and Calibration: Self-Calibration of Consumer Depth 
Cameras, CVPR 2014
Qian-Yi Zhou and Vladlen Koltun

Project page:
http://www.stanford.edu/~qianyizh/projects/elasticreconstruction.html
http://www.stanford.edu/~qianyizh/projects/scene.html
Data page:
http://www.stanford.edu/~qianyizh/projects/scenedata.html

This github repository is maintained by Qian-Yi Zhou (Qianyi.Zhou@gmail.com)
Contact me or Vladlen Koltun (vkoltun@gmail.com) if you have any questions.

===============================================================================
License

The source code is released under MIT license.

In general, you can do anything with the code for any purposes, with proper 
attribution. If you do something interesting with the code, we'll be happy to 
know about it. Feel free to contact us: Qian-Yi Zhou (Qianyi.Zhou@gmail.com), 
Vladlen Koltun (vkoltun@gmail.com).

The data on our data page can be used for any purposes with proper attribution.
I scanned the scenes and reconstructed them. Just use them if you like.

Please cite our ICCV 2013 paper "Elastic Fragments for Dense Scene
Reconstruction" and/or our CVPR 2014 paper "Simultaneous Localization and 
Calibration: Self-Calibration of Consumer Depth Cameras" if you use our code 
or data.

===============================================================================
Modules

+ FragmentOptimizer
This is the main function of this package. The purpose is to estimate an 
optimal nonrigid deformation for each fragment (individual point clouds) so
as to minimize the alignment error. It takes fragments, dense correspondences, 
and an initial camera pose trajectory as input, and outputs control lattices
for the fragments.

+ BuildCorrespondence
This is a pre-processing step of FragmentOptimizer. It takes fragments and an
initial camera pose trajectory as input, applies ICP registration and outputs
dense correspondences between aligned fragment pairs.

+ Integrate
This is the final step of the pipeline. It takes the depth stream, a rough
trajectory, and the deformed control lattices as the input, and output a voxel
representing the final geometry

===============================================================================
Quick Start

Download the data package from:

I will update this section very soon.

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
Compilation

We include project files for MSVS2010.

I will update this section very soon.

==============================================================================
