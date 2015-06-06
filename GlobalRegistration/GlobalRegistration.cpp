// GlobalRegistration.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include "helper.h"
#include "RansacCurvature.h"

// Types
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;


std::string dir_name = "";

void do_all( int num )
{
	Configuration config;

	RGBDTrajectory traj;
	RGBDInformation info;
#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
	for ( int i = 0; i < num; i++ ) {
		for ( int j = i + 1; j < num; j++ ) {
			// Load object and scene
			pcl::console::print_highlight( "Betweeen fragments %d and %d.\n", i, j );
			bool smart_swapped = false;

			PointCloudT::Ptr object (new PointCloudT);
			PointCloudT::Ptr object_aligned (new PointCloudT);
			PointCloudT::Ptr scene (new PointCloudT);
			FeatureCloudT::Ptr object_features (new FeatureCloudT);
			FeatureCloudT::Ptr scene_features (new FeatureCloudT);
			pcl::console::print_highlight ("Loading point clouds...\n");

			char filename[ 1024 ];
			sprintf( filename, "%scloud_bin_%d.pcd", dir_name.c_str(), i );
			pcl::io::loadPCDFile<PointNT>( filename, *scene );

			sprintf( filename, "%scloud_bin_%d.pcd", dir_name.c_str(), j );
			pcl::io::loadPCDFile<PointNT>( filename, *object );

			const float leaf = config.resample_leaf_;

			// Downsample
			pcl::console::print_highlight ("Downsampling...\n");
			pcl::VoxelGrid<PointNT> grid;
			grid.setLeafSize (leaf, leaf, leaf);
			grid.setInputCloud (object);
			grid.filter (*object);
			grid.setInputCloud (scene);
			grid.filter (*scene);

			if ( config.smart_swap_ ) {
				if ( object->size() > scene->size() ) {
					PointCloudT::Ptr temp = object;
					object = scene;
					scene = temp;
					smart_swapped = true;
				} else {
					smart_swapped = false;
				}
			}

			if ( config.estimate_normal_ ) {
				PointCloudT::Ptr scene_bak (new PointCloudT);
				pcl::copyPointCloud( *scene, *scene_bak );
				PointCloudT::Ptr object_bak (new PointCloudT);
				pcl::copyPointCloud( *object, *object_bak );

				// Estimate normals for scene
				pcl::console::print_highlight ("Estimating scene normals...\n");
				pcl::NormalEstimationOMP<PointNT,PointNT> nest;
				nest.setRadiusSearch( config.normal_radius_ );
				nest.setInputCloud (scene);
				nest.compute (*scene);
				for ( int i = 0; i < scene->size(); i++ ) {
					if ( scene->points[ i ].normal_x * scene_bak->points[ i ].normal_x 
						+ scene->points[ i ].normal_y * scene_bak->points[ i ].normal_y
						+ scene->points[ i ].normal_z * scene_bak->points[ i ].normal_z < 0.0 ) {
						scene->points[ i ].normal_x *= -1;
						scene->points[ i ].normal_y *= -1;
						scene->points[ i ].normal_z *= -1;
					}
				}

				// Estimate normals for object
				pcl::console::print_highlight ("Estimating object normals...\n");
				nest.setRadiusSearch( config.normal_radius_ );
				nest.setInputCloud (object);
				nest.compute (*object);
				for ( int i = 0; i < object->size(); i++ ) {
					if ( object->points[ i ].normal_x * object_bak->points[ i ].normal_x 
						+ object->points[ i ].normal_y * object_bak->points[ i ].normal_y
						+ object->points[ i ].normal_z * object_bak->points[ i ].normal_z < 0.0 ) {
						object->points[ i ].normal_x *= -1;
						object->points[ i ].normal_y *= -1;
						object->points[ i ].normal_z *= -1;
					}
				}
			}

			// Estimate features
			pcl::console::print_highlight ("Estimating features...\n");
			FeatureEstimationT fest;
			fest.setRadiusSearch ( config.feature_radius_ );
			fest.setInputCloud (object);
			fest.setInputNormals (object);
			fest.compute (*object_features);
			fest.setInputCloud (scene);
			fest.setInputNormals (scene);
			fest.compute (*scene_features);

			pcl::console::print_highlight ("Starting alignment...\n");
			RansacCurvature<PointNT,PointNT,FeatureT> align;
			align.setInputCloud (object);
			align.setSourceFeatures (object_features);
			align.setInputTarget (scene);
			align.setTargetFeatures (scene_features);
			align.setMaximumIterations ( config.max_iteration_ ); // Number of RANSAC iterations
			align.setNumberOfSamples ( config.num_of_samples_ ); // Number of points to sample for generating/prerejecting a pose
			align.setCorrespondenceRandomness ( config.correspondence_randomness_ ); // Number of nearest features to use
			align.setSimilarityThreshold ( config.edge_similarity_ ); // Polygonal edge length similarity threshold
			align.setMaxCorrespondenceDistance ( config.max_correspondence_distance_ ); // Inlier threshold
			align.setInlierFraction ( config.inlier_fraction_ ); // Required inlier fraction for accepting a pose hypothesis
			align.setInlierNumber( config.inlier_number_ );
			align.setAngleDiff( config.angle_difference_ );
			{
				pcl::ScopeTime t("Alignment");
				align.align (*object_aligned);
			}

			if (align.hasConverged ())
				//if (true)
			{
				align.getInformation();
				if ( config.aux_data_ ) {
					char test[ 1024 ];
					sprintf( test, "test_%d_%d.txt", i, j );
					align.writeAuxData( std::string( test ) );
				}
				Eigen::Matrix4f transformation;
				Eigen::Matrix< double, 6, 6 > information;
				if ( smart_swapped ) {
					transformation = align.getFinalTransformation ().inverse();
					information = align.information_target_;
				} else {
					transformation = align.getFinalTransformation ();
					information = align.information_source_;
				}
#pragma omp critical
				{
					// Print results
					pcl::console::print_info ("\n");
					pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
					pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
					pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
					pcl::console::print_info ("\n");
					pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
					pcl::console::print_info ("\n");
					pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
					traj.data_.push_back( FramedTransformation( i, j, num, transformation.cast< double >() ) );
					info.data_.push_back( FramedInformation( i, j, num, information ) );
				}
			} else {
				pcl::console::print_error ("Alignment failed!\n");
			}
		}
	}
	traj.SaveToFile( "result.txt" );
	info.SaveToFile( "result.info" );
}

int fragment;
RGBDTrajectory segment_traj;
RGBDTrajectory init_traj;
RGBDTrajectory pose_traj;
RGBDTrajectory odometry_traj;
RGBDInformation odometry_info;

void create_init_traj()
{
	init_traj.data_.clear();
	Eigen::Matrix4d base_mat = Eigen::Matrix4d::Identity();
	for ( size_t i = 0; i < segment_traj.data_.size(); i++ ) {
		if ( i % fragment == 0 && i > 0 ) {
			base_mat = init_traj.data_[ i - 1 ].transformation_ * segment_traj.data_[ i ].transformation_.inverse();
		}
		init_traj.data_.push_back( FramedTransformation( i, i, i + 1, base_mat * segment_traj.data_[ i ].transformation_ ) );
	}
	init_traj.SaveToFile( "init.log" );
}

void create_pose_traj()
{
	pose_traj.data_.clear();
	for ( size_t i = 0; i < init_traj.data_.size(); i += fragment ) {
		pose_traj.data_.push_back( FramedTransformation( i / fragment, i / fragment, i / fragment + 1, init_traj.data_[ i ].transformation_ * segment_traj.data_[ 0 ].transformation_.inverse() ) );
	}
	pose_traj.SaveToFile( "pose.log" );
}

void create_odometry( int num )
{
	Configuration config;
	odometry_traj.data_.clear();
	odometry_info.data_.clear();
	for ( int i = 1; i < num; i++ ) {
		odometry_traj.data_.push_back( FramedTransformation( i - 1, i, num, pose_traj.data_[ i - 1 ].transformation_.inverse() * pose_traj.data_[ i ].transformation_ ) );
		pcl::console::print_highlight( "Betweeen fragments %d and %d.\n", i - 1, i );
		PointCloudT::Ptr object (new PointCloudT);
		PointCloudT::Ptr object_aligned (new PointCloudT);
		PointCloudT::Ptr scene (new PointCloudT);
		FeatureCloudT::Ptr object_features (new FeatureCloudT);
		FeatureCloudT::Ptr scene_features (new FeatureCloudT);
		pcl::console::print_highlight ("Loading point clouds...\n");

		char filename[ 1024 ];
		sprintf( filename, "%scloud_bin_%d.pcd", dir_name.c_str(), i - 1 );
		pcl::io::loadPCDFile<PointNT>( filename, *scene );

		sprintf( filename, "%scloud_bin_%d.pcd", dir_name.c_str(), i );
		pcl::io::loadPCDFile<PointNT>( filename, *object );

		const float leaf = config.resample_leaf_;

		// Downsample
		pcl::console::print_highlight ("Downsampling...\n");
		pcl::VoxelGrid<PointNT> grid;
		grid.setLeafSize (leaf, leaf, leaf);
		grid.setInputCloud (object);
		grid.filter (*object);
		grid.setInputCloud (scene);
		grid.filter (*scene);

		if ( config.estimate_normal_ ) {
			PointCloudT::Ptr scene_bak (new PointCloudT);
			pcl::copyPointCloud( *scene, *scene_bak );
			PointCloudT::Ptr object_bak (new PointCloudT);
			pcl::copyPointCloud( *object, *object_bak );

			// Estimate normals for scene
			pcl::console::print_highlight ("Estimating scene normals...\n");
			pcl::NormalEstimationOMP<PointNT,PointNT> nest;
			nest.setRadiusSearch( config.normal_radius_ );
			nest.setInputCloud (scene);
			nest.compute (*scene);
			for ( int i = 0; i < scene->size(); i++ ) {
				if ( scene->points[ i ].normal_x * scene_bak->points[ i ].normal_x 
					+ scene->points[ i ].normal_y * scene_bak->points[ i ].normal_y
					+ scene->points[ i ].normal_z * scene_bak->points[ i ].normal_z < 0.0 ) {
					scene->points[ i ].normal_x *= -1;
					scene->points[ i ].normal_y *= -1;
					scene->points[ i ].normal_z *= -1;
				}
			}

			// Estimate normals for object
			pcl::console::print_highlight ("Estimating object normals...\n");
			nest.setRadiusSearch( config.normal_radius_ );
			nest.setInputCloud (object);
			nest.compute (*object);
			for ( int i = 0; i < object->size(); i++ ) {
				if ( object->points[ i ].normal_x * object_bak->points[ i ].normal_x 
					+ object->points[ i ].normal_y * object_bak->points[ i ].normal_y
					+ object->points[ i ].normal_z * object_bak->points[ i ].normal_z < 0.0 ) {
					object->points[ i ].normal_x *= -1;
					object->points[ i ].normal_y *= -1;
					object->points[ i ].normal_z *= -1;
				}
			}
		}

		// Estimate features
		pcl::console::print_highlight ("Estimating features...\n");
		FeatureEstimationT fest;
		fest.setRadiusSearch ( config.feature_radius_ );
		fest.setInputCloud (object);
		fest.setInputNormals (object);
		fest.compute (*object_features);
		fest.setInputCloud (scene);
		fest.setInputNormals (scene);
		fest.compute (*scene_features);

		pcl::console::print_highlight ("Starting alignment...\n");
		RansacCurvature<PointNT,PointNT,FeatureT> align;
		align.setInputCloud (object);
		align.setSourceFeatures (object_features);
		align.setInputTarget (scene);
		align.setTargetFeatures (scene_features);
		align.setMaximumIterations ( config.max_iteration_ ); // Number of RANSAC iterations
		align.setNumberOfSamples ( config.num_of_samples_ ); // Number of points to sample for generating/prerejecting a pose
		align.setCorrespondenceRandomness ( config.correspondence_randomness_ ); // Number of nearest features to use
		align.setSimilarityThreshold ( config.edge_similarity_ ); // Polygonal edge length similarity threshold
		align.setMaxCorrespondenceDistance ( config.max_correspondence_distance_ ); // Inlier threshold
		align.setInlierFraction ( config.inlier_fraction_ ); // Required inlier fraction for accepting a pose hypothesis
		align.setInlierNumber( config.inlier_number_ );
		align.setAngleDiff( config.angle_difference_ );
		{
			pcl::ScopeTime t("Alignment");
			//align.align (*object_aligned);
			align.align_redux( *object_aligned, odometry_traj.data_[ i - 1 ].transformation_.cast< float >() );
		}

		align.getInformation();
		odometry_info.data_.push_back( FramedInformation( i - 1, i, num, align.information_source_ ) );
	}
	odometry_traj.SaveToFile( "odometry.log" );
	odometry_info.SaveToFile( "odometry.info" );
}

int main(int argc, char * argv[])
{
	if ( argc < 2 ) {
		cout << "Usage : " << endl;
		cout << "    GlobalRegistration.exe <dir>" << endl;
		cout << "    GlobalRegistration.exe <dir> <100-0.log> <segment_length>" << endl;
		return 0;
	}
	dir_name = std::string( argv[ 1 ] );
	int num_of_pcds =  std::count_if( boost::filesystem::directory_iterator( boost::filesystem::path( dir_name ) ),
		boost::filesystem::directory_iterator(), 
		[](const boost::filesystem::directory_entry& e) {
			return e.path().extension() == ".pcd";  }
	);
	cout << num_of_pcds << " detected." << endl << endl;

	if ( argc == 2 ) {
		do_all( num_of_pcds );
	}

	if ( argc == 4 ) {
		segment_traj.LoadFromFile( argv[ 2 ] );
		sscanf( argv[ 3 ], "%d", &fragment );

		create_init_traj();
		create_pose_traj();
		create_odometry( num_of_pcds );

		if ( boost::filesystem::exists( "result.txt" ) ) {
			cout << "result.txt detected. skip global registration." << endl;
		} else {
			do_all( num_of_pcds );
		}
	}

	return 0;
}

