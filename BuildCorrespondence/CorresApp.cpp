#include "StdAfx.h"
#include "CorresApp.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <omp.h>
#include <boost/filesystem.hpp>

CCorresApp::CCorresApp(void)
	: save_xyzn_( false )
	, save_corres_( true )
	, dist_thresh_( 0.015 )
	, normal_thresh_( 0.8660 )
	, registration_( false )
	, output_information_( false )
	, reg_dist_( 0.03 )
	, reg_ratio_( 0.25 )
	, reg_num_( 40000 )
	, redux_( false )
	, num_( 0 )
	, bbox_length_( -1.0 )
	, length_( 3.0 )
	, interval_( 50 )
{
}


CCorresApp::~CCorresApp(void)
{
}

void CCorresApp::LoadData( std::string filename, int num )
{
	const char * c = strrchr( filename.c_str(), '\\' );
	if ( c == NULL ) {
		c = strrchr( filename.c_str(), '/' );
	}
	memset( m_pDirName, 0, 1024 );
	strncat_s( m_pDirName, 1024, filename.c_str(), c - filename.c_str() + 1 );

	if ( num > 0 ) {
		RGBDTrajectory temp;
		temp.LoadFromFile( filename );
		corres_traj_.data_.clear();
		num_ = num;

		Eigen::Matrix4d basepose = Eigen::Matrix4d::Identity();
		basepose( 0, 3 ) = length_ / 2.0;
		basepose( 1, 3 ) = length_ / 2.0;
		basepose( 2, 3 ) = - 0.3;

		Eigen::Matrix4d baseinverse = basepose.inverse();
		Eigen::Matrix4d leftbase = basepose * temp.data_[ 0 ].transformation_.inverse();
		std::vector< Eigen::Matrix4d > ipose;
		ipose.resize( num_ );

		for ( int i = 0; i < num_; i++ ) {
			ipose[ i ] = leftbase * temp.data_[ i * interval_ ].transformation_ * baseinverse;
		}

		for ( int i = 0; i < num_ - 1; i++ ) {
			corres_traj_.data_.push_back( FramedTransformation( i, i + 1, num_, ipose[ i ].inverse() * ipose[ i + 1 ] ) );
			for ( int j = i + 2; j < num; j++ ) {
				Eigen::Matrix4d trans = ipose[ i ].inverse() * ipose[ j ];
				if ( GetVolumeOverlapRatio( trans ) > 0.3 ) {
					corres_traj_.data_.push_back( FramedTransformation( i, j, num_, trans ) );
				}
			}
		}
		PCL_WARN( "%d initial matching candidates are created.\n", corres_traj_.data_.size() );
	} else {
		corres_traj_.LoadFromFile( filename );
		num_ = corres_traj_.data_[ 0 ].frame_;
	}

	pointclouds_.clear();
	pointclouds_.resize( num_ );

	if ( bbox_length_ > 0.0 ) {
		PCL_INFO( "BBox prune enabled : %.6f\n", bbox_length_ );
	}

	#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
	for ( int i = 0; i < num_; i++ ) {
		char fn[ 1024 ];
		memset( fn, 0, 1024 );
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rawpcd( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcd( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
		sprintf( fn, "%scloud_bin_%d.pcd",m_pDirName, i );
		PCL_INFO( "Load file : %s\n", fn );
		if ( pcl::io::loadPCDFile( fn, *rawpcd ) < 0 ) {
			PCL_ERROR( "Error loading file.\n" );
		}
		for ( int j = 0; j < ( int )rawpcd->points.size(); j++ ) {
			if ( !_isnan( rawpcd->points[ j ].normal_x ) ) {
				pcd->push_back( rawpcd->points[ j ] );
			}
		}
		pointclouds_[ i ] = pcd;

		if ( save_xyzn_ ) {
			sprintf( fn, "%scloud_bin_xyzn_%d.xyzn", m_pDirName, i );
			FILE * f = fopen( fn, "w" );
			for ( int k = 0; k < ( int )pcd->size(); k++ ) {
				pcl::PointXYZRGBNormal & pt = pcd->points[ k ];
				fprintf( f, "%.6f %.6f %.6f %.6f %.6f %.6f\n", pt.x, pt.y, pt.z, pt.normal_x, pt.normal_y, pt.normal_z );
			}
			fclose( f );
		}
	}
}

void CCorresApp::FindCorrespondence()
{
	if ( output_information_ ) {
		corres_info_.data_.clear();
		for ( int i = 0; i < ( int )corres_traj_.data_.size(); i++ ) {
			corres_info_.data_.push_back( FramedInformation( corres_traj_.data_[ i ].id1_, corres_traj_.data_[ i ].id2_, corres_traj_.data_[ i ].frame_, InformationMatrix::Zero() ) );
		}
	}

	#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
	for ( int i = 0; i < ( int )corres_traj_.data_.size(); i++ ) {
		if ( blacklist_.find( corres_traj_.data_[ i ].id1_ ) != blacklist_.end() || blacklist_.find( corres_traj_.data_[ i ].id2_ ) != blacklist_.end() ) {
			continue;
		}

		std::vector< CorrespondencePair > corres;
		int old_id = -1;
		pcl::KdTreeFLANN< pcl::PointXYZRGBNormal > tree;
		const int K = 1;
		std::vector< int > pointIdxNKNSearch(K);
		std::vector< float > pointNKNSquaredDistance(K);

		if ( corres_traj_.data_[ i ].frame_ == -1 ) {
			continue;			// means reject
		}

		PCL_INFO( "Processing pair <%d, %d>\n", corres_traj_.data_[ i ].id1_, corres_traj_.data_[ i ].id2_ );

		corres.clear();
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcd0 = pointclouds_[ corres_traj_.data_[ i ].id1_ ];
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcd1 = pointclouds_[ corres_traj_.data_[ i ].id2_ ];

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformed( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
		pcl::transformPointCloudWithNormals( *pcd1, *transformed, corres_traj_.data_[ i ].transformation_ );

		if ( old_id != corres_traj_.data_[ i ].id1_ ) {
			tree.setInputCloud( pcd0 );
			old_id = corres_traj_.data_[ i ].id1_;
		}

		for ( int k = 0; k < ( int )transformed->size(); k++ ) {
			if ( tree.nearestKSearch( transformed->points[ k ], K, pointIdxNKNSearch, pointNKNSquaredDistance ) > 0 ) {
				if ( pointNKNSquaredDistance[ 0 ] < dist_thresh_ * dist_thresh_ && 
					NormalDot( pcd0->points[ pointIdxNKNSearch[ 0 ] ], transformed->points[ k ] ) > normal_thresh_ ) {

					corres.push_back( CorrespondencePair( pointIdxNKNSearch[ 0 ], k ) );

				}
			}
		}

		PCL_INFO( "    <%d, %d> : Corresponce number is %d, ratio is %.2f(%d)\n", corres_traj_.data_[ i ].id1_, corres_traj_.data_[ i ].id2_, corres.size(), ( double )corres.size() / ( double )corres_traj_.data_[ i ].frame_, corres_traj_.data_[ i ].frame_ );
		if ( ( double )corres.size() / ( double )corres_traj_.data_[ i ].frame_ < 0.5 ) {
			PCL_WARN( "    <%d, %d> : Reduced too much!!\n", corres_traj_.data_[ i ].id1_, corres_traj_.data_[ i ].id2_ );
			if ( reg_num_ > 0 ) {
				corres_traj_.data_[ i ].frame_ = -1;
			} else {
				corres_traj_.data_[ i ].frame_ = ( int )corres.size();
			}
		} else {
			corres_traj_.data_[ i ].frame_ = ( int )corres.size();
		}

		if ( save_corres_ ) {
			char fn[ 1024 ];
			memset( fn, 0, 1024 );
			sprintf( fn, "%scorres_%d_%d.txt", m_pDirName, corres_traj_.data_[ i ].id1_, corres_traj_.data_[ i ].id2_ );
			FILE * f = fopen( fn, "w" );
			for ( int k = 0; k < ( int )corres.size(); k++ ) {
				fprintf( f, "%d %d\n", corres[ k ].first, corres[ k ].second );
			}
			fclose( f );
		}

		if ( output_information_ ) {
			corres_info_.data_[ i ].frame_ = corres_traj_.data_[ i ].frame_;

			InformationMatrix & ATA = corres_info_.data_[ i ].information_;
			ATA.setZero ();

			for ( int k = 0; k < ( int )corres.size(); k++ ) {
				pcl::PointXYZRGBNormal * source_it = &( pcd1->points[ corres[ k ].second ] );
				const float & sx = source_it->x;
				const float & sy = source_it->y;
				const float & sz = source_it->z;

				Eigen::Matrix< double, 3, 6 > A;
				A << 1, 0, 0, 0, 2 * sz, - 2 * sy,
					0, 1, 0, - 2 * sz, 0, 2 * sx,
					0, 0, 1, 2 * sy, - 2 * sx, 0;

				ATA += A.transpose() * A;
			}

			//cout << ATA << endl << endl;
			corres_info_.data_[ i ].information_ = ATA;
		}
	}
}

void CCorresApp::Registration()
{
	registration_ = true;
	PCL_WARN( "Registration with dist %.6f, num %d and ratio %.6f\n", reg_dist_, reg_num_, reg_ratio_ );

	int nprocessed = 0;
	//omp_set_num_threads( 8 );

	#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
	for ( int i = 0; i < ( int )corres_traj_.data_.size(); i++ ) {
		if ( blacklist_.find( corres_traj_.data_[ i ].id1_ ) != blacklist_.end() || blacklist_.find( corres_traj_.data_[ i ].id2_ ) != blacklist_.end() ) {
			#pragma omp atomic
			nprocessed++;
			corres_traj_.data_[ i ].frame_ = -1;
			PCL_INFO( "Blacklist pair <%d, %d> ... \n", corres_traj_.data_[ i ].id1_, corres_traj_.data_[ i ].id2_ );
			PCL_WARN( "%d / %d\n", nprocessed, corres_traj_.data_.size() );
			continue;
		}
		if ( corres_traj_.data_[ i ].frame_ == -1 ) {
			#pragma omp atomic
			nprocessed++;
			continue;			// means reject
		}

		std::vector< CorrespondencePair > corres;
		int old_id = -1;
		pcl::KdTreeFLANN< pcl::PointXYZRGBNormal > tree;
		const int K = 1;
		std::vector< int > pointIdxNKNSearch(K);
		std::vector< float > pointNKNSquaredDistance(K);

		PCL_INFO( "Align pair <%d, %d> ... \n", corres_traj_.data_[ i ].id1_, corres_traj_.data_[ i ].id2_ );

		corres.clear();
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcd0 = pointclouds_[ corres_traj_.data_[ i ].id1_ ];
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcd1 = pointclouds_[ corres_traj_.data_[ i ].id2_ ];

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformed( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
		pcl::transformPointCloudWithNormals( *pcd1, *transformed, corres_traj_.data_[ i ].transformation_ );

		if ( old_id != corres_traj_.data_[ i ].id1_ ) {
			tree.setInputCloud( pcd0 );
			old_id = corres_traj_.data_[ i ].id1_;
		}

		int cnt = 0;
		for ( int k = 0; k < ( int )transformed->size(); k++ ) {
			if ( tree.nearestKSearch( transformed->points[ k ], K, pointIdxNKNSearch, pointNKNSquaredDistance ) > 0 ) {
				if ( pointNKNSquaredDistance[ 0 ] < reg_dist_ * reg_dist_ ) {
					cnt++;
				}
			}
		}

		double r1 = ( double )cnt / ( double )pcd0->size();
		double r2 = ( double )cnt / ( double )transformed->size();
		PCL_INFO( "    <%d, %d> : %d inliers with ratio %.2f(%d) and %.2f(%d) ... ", corres_traj_.data_[ i ].id1_, corres_traj_.data_[ i ].id2_, cnt, r1, pcd0->size(), r2, transformed->size() );
//		bool accept = ( corres_traj_.data_[ i ].id2_ - corres_traj_.data_[ i ].id1_ == 1 || cnt >= reg_num_ || ( r1 > reg_ratio_ && r2 > reg_ratio_ ) );
		bool accept = ( cnt >= reg_num_ || ( r1 > reg_ratio_ && r2 > reg_ratio_ ) );
		if ( accept ) {
			PCL_INFO( "accept.\n" );
			corres_traj_.data_[ i ].frame_ = cnt;
		} else {
			PCL_INFO( "reject.\n" );
			corres_traj_.data_[ i ].frame_ = -1;
			#pragma omp atomic
			nprocessed++;
			PCL_WARN( "%d / %d\n", nprocessed, corres_traj_.data_.size() );
			continue;
		}

		if ( redux_ ) {
			stdext::hash_map< int, int >::iterator it = redux_map_.find( GetReduxIndex( corres_traj_.data_[ i ].id1_, corres_traj_.data_[ i ].id2_ ) );

			if ( it != redux_map_.end() ) {
				corres_traj_.data_[ i ].transformation_ = redux_traj_.data_[ it->second ].transformation_;
				#pragma omp atomic
				nprocessed++;
				PCL_WARN( "%d / %d\n", nprocessed, corres_traj_.data_.size() );
				continue;
			}
		}

		pcl::IterativeClosestPoint< pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal > icp;				
		typedef pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> PointToPlane;
		boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);

		icp.setInputCloud( pcd1 );
		icp.setInputTarget( pcd0 );
		icp.setMaxCorrespondenceDistance( reg_dist_ );
		icp.setMaximumIterations( 20 );
		icp.setTransformationEpsilon( 1e-6 );
		icp.setTransformationEstimation( point_to_plane );

		icp.align( *transformed, corres_traj_.data_[ i ].transformation_.cast<float>() );
		PCL_INFO( "    <%d, %d> : ICP fitness score is %.6f\n", corres_traj_.data_[ i ].id1_, corres_traj_.data_[ i ].id2_, icp.getFitnessScore() );
		PCL_INFO( "    Matrix from : \n" );
		cout << corres_traj_.data_[ i ].transformation_ << endl;
		PCL_INFO( "    To : \n" );
		cout << icp.getFinalTransformation() << endl;
		corres_traj_.data_[ i ].transformation_ = icp.getFinalTransformation().cast<double>();

		#pragma omp atomic
		nprocessed++;

		PCL_WARN( "%d / %d\n", nprocessed, corres_traj_.data_.size() );
	}
}

void CCorresApp::Finalize()
{
	corres_traj_.SaveToFile( "reg_output.log" );

	if ( output_information_ ) {
		corres_info_.SaveToFile( "reg_output.info" );
	}
}

void CCorresApp::Blacklist( std::string filename )
{
	blacklist_.clear();

	FILE * f = fopen( filename.c_str(), "r" );
	if ( f != NULL ) {
		char buffer[1024];
		int id;
		while ( fgets( buffer, 1024, f ) != NULL ) {
			if ( strlen( buffer ) > 0 && buffer[ 0 ] != '#' ) {
				sscanf( buffer, "%d", &id);
				blacklist_.insert( id );
			}
		}
		fclose ( f );
	}
}

void CCorresApp::Redux( std::string filename )
{
	redux_ = true;
	redux_map_.clear();
	redux_traj_.LoadFromFile( filename );

	for ( int i = 0; i < ( int )redux_traj_.data_.size(); i++ ) {
		redux_map_.insert( IntPair( GetReduxIndex( redux_traj_.data_[ i ].id1_, redux_traj_.data_[ i ].id2_ ), i ) );
	}

	PCL_WARN( "%d out of %d pairs are redux pairs.\n", redux_traj_.data_.size(), corres_traj_.data_.size() );
}
