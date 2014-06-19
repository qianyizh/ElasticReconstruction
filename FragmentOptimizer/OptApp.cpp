#include "StdAfx.h"
#include "OptApp.h"
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <boost/filesystem.hpp>

COptApp::COptApp(void)
	: resolution_( 8 )
	, interval_( 50 )
	, num_( 0 )
	, weight_( 1.0f )
	, length_( 3.0f )
	, max_iteration_( 5 )
	, max_inner_iteration_( 10 )
	, ctr_filename_( "output.ctr" )
	, sample_filename_( "sample.pcd" )
	, pose_filename_( "pose.log" )
	, init_ctr_file_( "" )
	, dir_prefix_( "" )
	, sample_num_( -1 )
	, blacklist_pair_num_( 10000 )
{
}

COptApp::~COptApp(void)
{
}

void COptApp::InitMap()
{
	absolute2relative_map_.resize( num_ );
	relative2absolute_map_.clear();
	for ( int i = 0; i < num_; i++ ) {
		if ( blacklist_.find( i ) == blacklist_.end() ) {
			absolute2relative_map_[ i ] = relative2absolute_map_.size();
			relative2absolute_map_.push_back( i );
		} else {
			absolute2relative_map_[ i ] = -1;
		}
	}
	if ( num_ != relative2absolute_map_.size() ) {
		PCL_WARN( "Blacklisted fragments, number reduced from %d to %d.\n", num_, relative2absolute_map_.size() );
	}
	num_ = relative2absolute_map_.size();
}

void COptApp::InitIPose()
{
	if ( ipose_.size() > 0 ) {
		return;
	}

	Eigen::Matrix4d basepose = Eigen::Matrix4d::Identity();
	basepose( 0, 3 ) = length_ / 2.0;
	basepose( 1, 3 ) = length_ / 2.0;
	basepose( 2, 3 ) = - 0.3;
	
	Eigen::Matrix4d baseinverse = basepose.inverse();

	Eigen::Matrix4d leftbase = basepose * rgbd_traj_.data_[ 0 ].transformation_.inverse();
	ipose_.resize( num_ );
	pose_.resize( num_ );
	pose_rot_t_.resize( num_ );

	for ( int i = 0; i < num_; i++ ) {
		int ii = relative2absolute_map_[ i ];
		ipose_[ i ] = leftbase * rgbd_traj_.data_[ ii * interval_ ].transformation_ * baseinverse;
	}
	PCL_INFO( "IPose initialized.\n" );
}

void COptApp::InitPointClouds()
{
	pointclouds_.clear();
	for ( int i = 0; i < num_; i++ ) {
		pointclouds_.push_back( PointCloud( i, resolution_, length_ ) );
	}

	#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
	for ( int i = 0; i < num_; i++ ) {
		int ii = relative2absolute_map_[ i ];
		char filename[ 1024 ];
		memset( filename, 0, 1024 );
		sprintf( filename, "%scloud_bin_%d.pcd", dir_prefix_.c_str(), ii );
		if ( boost::filesystem::exists( std::string( filename ) ) ) {
			pointclouds_[ i ].LoadFromPCDFile( filename );
		} else {
			sprintf( filename, "%scloud_bin_xyzn_%d.xyzn", dir_prefix_.c_str(), ii );
			if ( boost::filesystem::exists( std::string( filename ) ) ) {
				pointclouds_[ i ].LoadFromXYZNFile( filename );
			} else {
				PCL_ERROR( "File not found ... Check dir and num parameters.\n" );
			}
		}
	}
}

void COptApp::InitCorrespondences()
{
	corres_.clear();
	for ( int i = 0; i < ( int )reg_traj_.data_.size(); i++ ) {
		if ( blacklist_.find( reg_traj_.data_[ i ].id1_ ) != blacklist_.end() || blacklist_.find( reg_traj_.data_[ i ].id2_ ) != blacklist_.end()
			|| reg_traj_.data_[ i ].id1_ >= num_ || reg_traj_.data_[ i ].id2_ >= num_ ) {
			continue;
		}
		if ( reg_traj_.data_[ i ].frame_ != -1 && reg_traj_.data_[ i ].frame_ >= blacklist_pair_num_ ) {
			corres_.push_back( Correspondence( absolute2relative_map_[ reg_traj_.data_[ i ].id1_ ], absolute2relative_map_[ reg_traj_.data_[ i ].id2_ ] ) );
			char filename[ 1024 ];
			memset( filename, 0, 1024 );
			sprintf( filename, "%scorres_%d_%d.txt", dir_prefix_.c_str(), reg_traj_.data_[ i ].id1_, reg_traj_.data_[ i ].id2_ );
			corres_.back().trans_ = reg_traj_.data_[ i ].transformation_;
			corres_.back().LoadFromFile( filename );
			PCL_INFO( "Read %s, get %d correspondences.\n", filename, corres_.back().corres_.size() );
		}
	}
}

void COptApp::OptimizeNonrigid()
{
	PCL_WARN( "Nonrigid optimization.\n" );
	PCL_INFO( "Parameters: weight %.5f, resolution %d, piece number %d, max iteration %d\n", weight_, resolution_, num_, max_iteration_ );

	InitMap();
	unit_length_ = length_ / resolution_;
	nper_ = ( resolution_ + 1 ) * ( resolution_ + 1 ) * ( resolution_ + 1 ) * 3;
	matrix_size_ = nper_ * num_;
	InitIPose();
	InitPointClouds();
	InitCorrespondences();

	Eigen::VectorXd ctr( matrix_size_ );
	Eigen::VectorXd ictr( matrix_size_ );
	Eigen::VectorXd oldctr( matrix_size_ );
	Eigen::VectorXd Ab( matrix_size_ );
	SparseMatrix baseAA( matrix_size_, matrix_size_ );
	SparseMatrix thisAA( matrix_size_, matrix_size_ );

	// init ctr
	InitCtr( ctr );
	ictr = ctr;
	oldctr = ctr;
	InitBaseAA( baseAA );

	double score;

	// iteration
	for ( int itr = 0; itr < max_iteration_; itr++ ) {
		int nprocessed = 0;
		for ( int l = 0; l < num_; l++ ) {
			pointclouds_[ l ].UpdateAllNormal( ctr );
		}

		thisAA = baseAA;

		PCL_INFO( "Processing %d : %5d", corres_.size(), 0 );

#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
		for ( int l = 0; l < ( int )corres_.size(); l++ ) {
			int i = corres_[ l ].idx0_;
			int j = corres_[ l ].idx1_;
			TripletVector tri;
			HashSparseMatrix mati( i * nper_, i * nper_ );
			HashSparseMatrix matj( j * nper_, j * nper_ );
			HashSparseMatrix matij( i * nper_, j * nper_ );

			int idx1[ 24 ], idx2[ 24 ];
			double val1[ 24 ], val2[ 24 ];

			for ( int k = 0; k < ( int )corres_[ l ].corres_.size(); k++ ) {
				int ii = corres_[ l ].corres_[ k ].first;
				int jj = corres_[ l ].corres_[ k ].second;
				Point & pi = pointclouds_[ i ].points_[ ii ];
				Point & pj = pointclouds_[ j ].points_[ jj ];

				for ( int t = 0; t < 8; t++ ) {
					idx1[ t ] = pi.idx_[ t ];
					val1[ t ] = pi.val_[ t ] * weight_ * pi.n_[ 0 ];
					idx1[ 8 + t ] = pi.idx_[ t ] + 1;
					val1[ 8 + t ] = pi.val_[ t ] * weight_ * pi.n_[ 1 ];
					idx1[ 16 + t ] = pi.idx_[ t ] + 2;
					val1[ 16 + t ] = pi.val_[ t ] * weight_ * pi.n_[ 2 ];
					idx2[ t ] = pj.idx_[ t ];
					val2[ t ] = - pj.val_[ t ] * weight_ * pi.n_[ 0 ];
					idx2[ 8 + t ] = pj.idx_[ t ] + 1;
					val2[ 8 + t ] = - pj.val_[ t ] * weight_ * pi.n_[ 1 ];
					idx2[ 16 + t ] = pj.idx_[ t ] + 2;
					val2[ 16 + t ] = - pj.val_[ t ] * weight_ * pi.n_[ 2 ];
				}

				mati.AddHessian( idx1, val1, 24, tri );
				matj.AddHessian( idx2, val2, 24, tri );
				matij.AddHessian( idx1, val1, 24, idx2, val2, 24, tri );
			}

			SparseMatrix tempAA( matrix_size_, matrix_size_ );
			tempAA.setFromTriplets( tri.begin(), tri.end() );

#pragma omp critical
			{
				nprocessed++;
				thisAA += tempAA;
				PCL_INFO( "\b\b\b\b\b%5d", nprocessed );
			}
		}
		PCL_INFO( " ... Done.\n" );

		Eigen::CholmodSupernodalLLT< SparseMatrix, Eigen::Upper > solver;
		solver.analyzePattern( thisAA );
		solver.factorize( thisAA );

		if ( sample_num_ > 0 ) {
			char filename[ 1024 ];
			memset( filename, 0, 1024 );
			sprintf( filename, "itr%d.ctr", itr );
			SaveCtr( oldctr, filename );
		}

		for ( int m = 0; m < max_inner_iteration_; m++ ) {
			Ab.setZero();
			for ( int l = 0; l < num_; l++ ) {
				for ( int i = 0; i <= resolution_; i++ ) {
					for ( int j = 0; j <= resolution_; j++ ) {
						for ( int k = 0; k <= resolution_; k++ ) {
							int idx = GetIndex( i, j, k ) * 3 + l * nper_;
							std::vector< int > idxx;
							if ( i > 0 ) {
								idxx.push_back( GetIndex( i - 1, j, k ) * 3 + l * nper_ );
							}
							if ( i < resolution_) {
								idxx.push_back( GetIndex( i + 1, j, k ) * 3 + l * nper_ );
							}
							if ( j > 0 ) {
								idxx.push_back( GetIndex( i, j - 1, k ) * 3 + l * nper_ );
							}
							if ( j < resolution_) {
								idxx.push_back( GetIndex( i, j + 1, k ) * 3 + l * nper_ );
							}
							if ( k > 0 ) {
								idxx.push_back( GetIndex( i, j, k - 1 ) * 3 + l * nper_ );
							}
							if ( k < resolution_) {
								idxx.push_back( GetIndex( i, j, k + 1 ) * 3 + l * nper_ );
							}
							Eigen::Matrix3d R = GetRotation( idx, idxx, ictr, ctr );
							for ( int t = 0; t < ( int )idxx.size(); t++ ) {
								Eigen::Vector3d bx = R * Eigen::Vector3d( ictr( idx ) - ictr( idxx[ t ] ), ictr( idx + 1 ) - ictr( idxx[ t ] + 1 ), ictr( idx + 2 ) - ictr( idxx[ t ] + 2 ) );
								Ab( idx ) += bx( 0 );
								Ab( idxx[ t ] ) -= bx( 0 );
								Ab( idx + 1 ) += bx( 1 );
								Ab( idxx[ t ] + 1 ) -= bx( 1 );
								Ab( idx + 2 ) += bx( 2 );
								Ab( idxx[ t ] + 2 ) -= bx( 2 );
							}
						}
					}
				}
			}
		
			oldctr = ctr;
			ctr = solver.solve( Ab );

			score = ( oldctr - ctr ).norm();
			PCL_WARN( "Iteration #%d:%d (%d:%d) : score is %.4f\n", itr + 1, m + 1, max_iteration_, max_inner_iteration_, score );

			if ( sample_num_ > 0 ) {
				char filename[ 1024 ];
				memset( filename, 0, 1024 );
				sprintf( filename, "itr%d_inner%d_out.ctr", itr, m );
				SaveCtr( ctr, filename );
			}
		}
	}

	SaveCtr( ctr, ctr_filename_ );
	SavePoints( ctr, sample_filename_ );
}

void COptApp::OptimizeRigid()
{
	PCL_WARN( "Rigid optimization.\n" );
	PCL_INFO( "Parameters: resolution %d, piece number %d, max iteration %d\n", resolution_, num_, max_iteration_ );

	InitMap();
	unit_length_ = length_ / resolution_;
	nper_ = ( resolution_ + 1 ) * ( resolution_ + 1 ) * ( resolution_ + 1 ) * 3;
	matrix_size_ = 6 * num_;			// rigid only
	InitIPose();
	InitPointClouds();
	InitCorrespondences();

	rgbd_traj_.data_.clear();
	for ( int i = 0; i < ( int )ipose_.size(); i++ ) {
		rgbd_traj_.data_.push_back( FramedTransformation( i, i, i + 1, ipose_[ i ] ) );
		pose_[ i ] = ipose_[ i ];
		pointclouds_[ i ].UpdatePose( pose_[ i ].cast< float >() );
	}

	for ( int itr = 0; itr < max_iteration_; itr++ ) {
		Eigen::VectorXd thisJb( matrix_size_ );
		SparseMatrix thisJJ( matrix_size_, matrix_size_ );

		thisJb.setZero();
		thisJJ.setZero();

		double thisscore = 0.0;
		int nprocessed = 0;

		PCL_INFO( "Processing %d : %5d", corres_.size(), 0 );

#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
		for ( int l = 0; l < ( int )corres_.size(); l++ ) {
			int i = corres_[ l ].idx0_;
			int j = corres_[ l ].idx1_;
			TripletVector tri;
			HashSparseMatrix mat_adder( 0, 0 );
			Eigen::VectorXd tempJb( matrix_size_ );
			SparseMatrix tempJJ( matrix_size_, matrix_size_ );
			tempJb.setZero();
			tempJJ.setZero();

			for ( int k = 0; k < 6; k++ ) {
				mat_adder.Add( k, k, 1, tri );
			}

			int idx[ 12 ];
			double val[ 12 ];
			double b;
			double score = 0.0;

			for ( int k = 0; k < ( int )corres_[ l ].corres_.size(); k++ ) {
				int ii = corres_[ l ].corres_[ k ].first;
				int jj = corres_[ l ].corres_[ k ].second;
				Point & pi = pointclouds_[ i ].points_[ ii ];
				Point & pj = pointclouds_[ j ].points_[ jj ];
				Eigen::Vector4d ppi( pi.p_[ 0 ], pi.p_[ 1 ], pi.p_[ 2 ], 1.0 );
				Eigen::Vector4d ppj( pj.p_[ 0 ], pj.p_[ 1 ], pj.p_[ 2 ], 1.0 );
				Eigen::Vector4d npi( pi.n_[ 0 ], pi.n_[ 1 ], pi.n_[ 2 ], 0.0 );
				Eigen::Vector4d npj( pj.n_[ 0 ], pj.n_[ 1 ], pj.n_[ 2 ], 0.0 );

				b = ( ppi - ppj ).dot( npi );
				score += b * b;

				idx[ 0 ] = i * 6;
				idx[ 1 ] = i * 6 + 1;
				idx[ 2 ] = i * 6 + 2;
				idx[ 3 ] = i * 6 + 3;
				idx[ 4 ] = i * 6 + 4;
				idx[ 5 ] = i * 6 + 5;
				idx[ 6 ] = j * 6;
				idx[ 7 ] = j * 6 + 1;
				idx[ 8 ] = j * 6 + 2;
				idx[ 9 ] = j * 6 + 3;
				idx[ 10 ] = j * 6 + 4;
				idx[ 11 ] = j * 6 + 5;

				val[ 0 ] = Eigen::Vector4d( 0, -ppi( 2 ), ppi( 1 ), 1 ).dot( npi ) + Eigen::Vector4d( 0, -npi( 2 ), npi( 1 ), 0 ).dot( ppi - ppj );
				val[ 1 ] = Eigen::Vector4d( ppi( 2 ), 0, -ppi( 0 ), 1 ).dot( npi ) + Eigen::Vector4d( npi( 2 ), 0, -npi( 0 ), 0 ).dot( ppi - ppj );
				val[ 2 ] = Eigen::Vector4d( -ppi( 1 ), ppi( 0 ), 0, 1 ).dot( npi ) + Eigen::Vector4d( -npi( 1 ), npi( 0 ), 0, 0 ).dot( ppi - ppj );
				val[ 3 ] = npi( 0 );
				val[ 4 ] = npi( 1 );
				val[ 5 ] = npi( 2 );
				val[ 6 ] = -Eigen::Vector4d( 0, -ppj( 2 ), ppj( 1 ), 1 ).dot( npi );
				val[ 7 ] = -Eigen::Vector4d( ppj( 2 ), 0, -ppj( 0 ), 1 ).dot( npi );
				val[ 8 ] = -Eigen::Vector4d( -ppj( 1 ), ppj( 0 ), 0, 1 ).dot( npi );
				val[ 9 ] = -npi( 0 );
				val[ 10 ] = -npi( 1 );
				val[ 11 ] = -npi( 2 );

				mat_adder.AddHessian( idx, val, 12, tri );
				mat_adder.AddJb( idx, val, 12, b, tempJb );
			}

			tempJJ.setFromTriplets( tri.begin(), tri.end() );

#pragma omp critical
			{
				nprocessed++;
				thisscore += score;
				thisJJ += tempJJ;
				thisJb += tempJb;
				PCL_INFO( "\b\b\b\b\b%5d", nprocessed );
			}
		}
		PCL_INFO( " ... Done.\n" );
		PCL_INFO( "Error score is : %.2f\n", thisscore );

		Eigen::CholmodSupernodalLLT< SparseMatrix, Eigen::Upper > solver;
		solver.analyzePattern( thisJJ );
		solver.factorize( thisJJ );

		Eigen::VectorXd result = - solver.solve( thisJb );

		for ( int l = 0; l < num_; l++ ) {
			Eigen::Affine3d aff_mat;
			aff_mat.linear() = ( Eigen::Matrix3d ) Eigen::AngleAxisd( result( l * 6 + 2 ), Eigen::Vector3d::UnitZ() )
				* Eigen::AngleAxisd( result( l * 6 + 1 ), Eigen::Vector3d::UnitY() )
				* Eigen::AngleAxisd( result( l * 6 + 0 ), Eigen::Vector3d::UnitX() );
			aff_mat.translation() = Eigen::Vector3d( result( l * 6 + 3 ), result( l * 6 + 4 ), result( l * 6 + 5 ) );
			pose_[ l ] = aff_mat.matrix() * pose_[ l ];
			pointclouds_[ l ].UpdatePose( aff_mat.matrix().cast< float >() );
		}
	}

	output_traj_.data_.clear();
	for ( int i = 0; i < ( int )pose_.size(); i++ ) {
		output_traj_.data_.push_back( FramedTransformation( i, i, i + 1, pose_[ i ] ) );
	}
	output_traj_.SaveToFile( pose_filename_ );

	Eigen::VectorXd ctr( nper_ * num_ );
	Pose2Ctr( pose_, ctr );
	SaveCtr( ctr, ctr_filename_ );
	SavePoints( ctr, sample_filename_ );
}

void COptApp::OptimizeSLAC()
{
	PCL_WARN( "SLAC optimization.\n" );
	PCL_INFO( "Parameters: weight %.5f, resolution %d, piece number %d, max iteration %d\n", weight_, resolution_, num_, max_iteration_ );

	double default_weight = num_ * weight_;

	InitMap();
	unit_length_ = length_ / resolution_;
	nper_ = ( resolution_ + 1 ) * ( resolution_ + 1 ) * ( resolution_ + 1 ) * 3;
	matrix_size_ = 6 * num_ + nper_;
	InitIPose();
	InitPointClouds();
	InitCorrespondences();

	Eigen::VectorXd ictr( matrix_size_ );
	Eigen::VectorXd thisCtr( matrix_size_ );
	InitCtrSLAC( ictr, thisCtr );
	Eigen::VectorXd expand_ctr( nper_ * num_ );

	rgbd_traj_.data_.clear();
	for ( int i = 0; i < ( int )ipose_.size(); i++ ) {
		rgbd_traj_.data_.push_back( FramedTransformation( i, i, i + 1, ipose_[ i ] ) );
		pose_[ i ] = ipose_[ i ];
		pose_rot_t_[ i ] = pose_[ i ].block< 3, 3 >( 0, 0 ).transpose();
		pointclouds_[ i ].UpdatePose( pose_[ i ].cast< float >() );
	}

	{
		pcl::ScopeTime ttime( "Neat Optimization" );

		SparseMatrix baseJJ( matrix_size_, matrix_size_ );
		InitBaseJJ( baseJJ );

		baseJJ *= default_weight;

		for ( int itr = 0; itr < max_iteration_; itr++ ) {
			Eigen::VectorXd thisJb( matrix_size_ );
			Eigen::MatrixXd thisJJ( baseJJ.triangularView< Eigen::Upper >() );
			Eigen::VectorXd result;

			thisJJ( 0, 0 ) += 1.0;
			thisJJ( 1, 1 ) += 1.0;
			thisJJ( 2, 2 ) += 1.0;
			thisJJ( 3, 3 ) += 1.0;
			thisJJ( 4, 4 ) += 1.0;
			thisJJ( 5, 5 ) += 1.0;

			thisJb.setZero();

			double thisscore = 0.0;
			int nprocessed = 0;

			PCL_INFO( "Processing %d : %5d", corres_.size(), 0 );

#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
			for ( int l = 0; l < ( int )corres_.size(); l++ ) {
				int i = corres_[ l ].idx0_;
				int j = corres_[ l ].idx1_;
				const int buck_size = 12 + 24 * 2;
				int idx[ buck_size ];
				double val[ buck_size ];
				double b;
				double score = 0.0;
				Eigen::MatrixXd tempJJ( matrix_size_, matrix_size_ );
				tempJJ.setZero();
				Eigen::VectorXd tempJb( matrix_size_ );
				tempJb.setZero();

				for ( int k = 0; k < ( int )corres_[ l ].corres_.size(); k++ ) {
					int ii = corres_[ l ].corres_[ k ].first;
					int jj = corres_[ l ].corres_[ k ].second;
					Point & pi = pointclouds_[ i ].points_[ ii ];
					Point & pj = pointclouds_[ j ].points_[ jj ];
					Eigen::Vector3d ppi( pi.p_[ 0 ], pi.p_[ 1 ], pi.p_[ 2 ] );
					Eigen::Vector3d ppj( pj.p_[ 0 ], pj.p_[ 1 ], pj.p_[ 2 ] );
					Eigen::Vector3d npi( pi.n_[ 0 ], pi.n_[ 1 ], pi.n_[ 2 ] );
					Eigen::Vector3d npj( pj.n_[ 0 ], pj.n_[ 1 ], pj.n_[ 2 ] );
					Eigen::Vector3d diff = ppi - ppj;
					b = diff.dot( npi );
					score += b * b;

					idx[ 0 ] = i * 6;
					idx[ 1 ] = i * 6 + 1;
					idx[ 2 ] = i * 6 + 2;
					idx[ 3 ] = i * 6 + 3;
					idx[ 4 ] = i * 6 + 4;
					idx[ 5 ] = i * 6 + 5;
					idx[ 6 ] = j * 6;
					idx[ 7 ] = j * 6 + 1;
					idx[ 8 ] = j * 6 + 2;
					idx[ 9 ] = j * 6 + 3;
					idx[ 10 ] = j * 6 + 4;
					idx[ 11 ] = j * 6 + 5;

					Eigen::Vector3d temp = ppj.cross( npi );

					val[ 0 ] = temp( 0 );
					val[ 1 ] = temp( 1 );
					val[ 2 ] = temp( 2 );
					val[ 3 ] = npi( 0 );
					val[ 4 ] = npi( 1 );
					val[ 5 ] = npi( 2 );
					val[ 6 ] = -temp( 0 );
					val[ 7 ] = -temp( 1 );
					val[ 8 ] = -temp( 2 );
					val[ 9 ] = -npi( 0 );
					val[ 10 ] = -npi( 1 );
					val[ 11 ] = -npi( 2 );

					// next part of Jacobian
					// deal with control lattices
					Eigen::Vector3d dTi = pose_rot_t_[ i ] * npi;
					Eigen::Vector3d dTj = - pose_rot_t_[ j ] * npi;

					for ( int ll = 0; ll < 8; ll++ ) {
						for ( int xyz = 0; xyz < 3; xyz++ ) {
							idx[ 12 + ll * 3 + xyz ] = 6 * num_ + pi.idx_[ ll ] + xyz;
							val[ 12 + ll * 3 + xyz ] = pi.val_[ ll ] * dTi( xyz );
						}
					}

					for ( int ll = 0; ll < 8; ll++ ) {
						for ( int xyz = 0; xyz < 3; xyz++ ) {
							idx[ 12 + 24 + ll * 3 + xyz ] = 6 * num_ + pj.idx_[ ll ] + xyz;
							val[ 12 + 24 + ll * 3 + xyz ] = pj.val_[ ll ] * dTj( xyz );
						}
					}

					for ( int i = 0; i < buck_size; i++ ) {
						tempJJ( idx[ i ], idx[ i ] ) += val[ i ] * val[ i ];
						for ( int j = i + 1; j < buck_size; j++ ) {
							if ( idx[ i ] == idx[ j ] ) {
								tempJJ( idx[ i ], idx[ j ] ) += 2 * val[ i ] * val[ j ];
							} else if ( idx[ i ] < idx[ j ] ) {
								tempJJ( idx[ i ], idx[ j ] ) += val[ i ] * val[ j ];
							} else {
								tempJJ( idx[ j ], idx[ i ] ) += val[ i ] * val[ j ];
							}
						}
						tempJb( idx[ i ] ) += b * val[ i ];
					}
				}

#pragma omp critical
				{
					nprocessed++;
					thisscore += score;
					thisJJ += tempJJ;
					thisJb += tempJb;
					PCL_INFO( "\b\b\b\b\b%5d", nprocessed );
				}
			}
			PCL_INFO( " ... Done.\n" );
			PCL_INFO( "Data error score is : %.2f\n", thisscore );

			SparseMatrix thisSparseJJ = thisJJ.sparseView();
			Eigen::CholmodSupernodalLLT< SparseMatrix, Eigen::Upper > solver;
			solver.analyzePattern( thisSparseJJ );
			solver.factorize( thisSparseJJ );

			Eigen::VectorXd tempCtr( thisCtr );

			// regularizer

			Eigen::VectorXd dataJb( thisJb );

			Eigen::VectorXd baseJb( matrix_size_ );
			baseJb.setZero();

			double regscore = 0.0;
			for ( int i = 0; i <= resolution_; i++ ) {
				for ( int j = 0; j <= resolution_; j++ ) {
					for ( int k = 0; k <= resolution_; k++ ) {
						int idx = GetIndex( i, j, k ) * 3 + 6 * num_;
						std::vector< int > idxx;
						if ( i > 0 ) {
							idxx.push_back( GetIndex( i - 1, j, k ) * 3 + 6 * num_ );
						}
						if ( i < resolution_) {
							idxx.push_back( GetIndex( i + 1, j, k ) * 3 + 6 * num_ );
						}
						if ( j > 0 ) {
							idxx.push_back( GetIndex( i, j - 1, k ) * 3 + 6 * num_ );
						}
						if ( j < resolution_) {
							idxx.push_back( GetIndex( i, j + 1, k ) * 3 + 6 * num_ );
						}
						if ( k > 0 ) {
							idxx.push_back( GetIndex( i, j, k - 1 ) * 3 + 6 * num_ );
						}
						if ( k < resolution_) {
							idxx.push_back( GetIndex( i, j, k + 1 ) * 3 + 6 * num_ );
						}

						Eigen::Matrix3d R;
						if ( i == resolution_ / 2 && j == resolution_ / 2 && k == 0 ) {
							// anchor point, Rotation matrix is always identity
							R = R.Identity();
						} else {
							R = GetRotation( idx, idxx, ictr, tempCtr );
						}

						for ( int t = 0; t < ( int )idxx.size(); t++ ) {
							Eigen::Vector3d bx = Eigen::Vector3d( tempCtr( idx ) - tempCtr( idxx[ t ] ), tempCtr( idx + 1 ) - tempCtr( idxx[ t ] + 1 ), tempCtr( idx + 2 ) - tempCtr( idxx[ t ] + 2 ) )
								- R * Eigen::Vector3d( ictr( idx ) - ictr( idxx[ t ] ), ictr( idx + 1 ) - ictr( idxx[ t ] + 1 ), ictr( idx + 2 ) - ictr( idxx[ t ] + 2 ) );
							regscore += default_weight * bx.transpose() * bx;
							baseJb( idx ) += bx( 0 ) * default_weight;
							baseJb( idxx[ t ] ) -= bx( 0 ) * default_weight;
							baseJb( idx + 1 ) += bx( 1 ) * default_weight;
							baseJb( idxx[ t ] + 1 ) -= bx( 1 ) * default_weight;
							baseJb( idx + 2 ) += bx( 2 ) * default_weight;
							baseJb( idxx[ t ] + 2 ) -= bx( 2 ) * default_weight;
						}
					}
				}
			}
			thisJb = dataJb + baseJb;

			PCL_INFO( "Regularization error score is : %.2f\n", regscore );

			result = - solver.solve( thisJb );

			for ( int l = 0; l < nper_; l++ ) {
				tempCtr( num_ * 6 + l ) = thisCtr( num_ * 6 + l ) + result( num_ * 6 + l );
			}

			for ( int l = 0; l < nper_; l++ ) {
				thisCtr( num_ * 6 + l ) += result( num_ * 6 + l );
			}

			for ( int l = 0; l < num_; l++ ) {
				Eigen::Affine3d aff_mat;
				aff_mat.linear() = ( Eigen::Matrix3d ) Eigen::AngleAxisd( result( l * 6 + 2 ), Eigen::Vector3d::UnitZ() )
					* Eigen::AngleAxisd( result( l * 6 + 1 ), Eigen::Vector3d::UnitY() )
					* Eigen::AngleAxisd( result( l * 6 + 0 ), Eigen::Vector3d::UnitX() );
				aff_mat.translation() = Eigen::Vector3d( result( l * 6 + 3 ), result( l * 6 + 4 ), result( l * 6 + 5 ) );
				//cout << aff_mat.matrix() << endl << endl;
				// update
				pose_[ l ] = aff_mat.matrix() * pose_[ l ];
				pose_rot_t_[ l ] = pose_[ l ].block< 3, 3 >( 0, 0 ).transpose();
			}
			ExpandCtr( thisCtr, expand_ctr );
			for ( int l = 0; l < num_; l++ ) {
				pointclouds_[ l ].UpdateAllPointPN( expand_ctr );
			}
		}
	}

	output_traj_.data_.clear();
	for ( int i = 0; i < ( int )pose_.size(); i++ ) {
		output_traj_.data_.push_back( FramedTransformation( i, i, i + 1, pose_[ i ] ) );
	}
	output_traj_.SaveToFile( pose_filename_ );

	ExpandCtr( thisCtr, expand_ctr );
	SaveCtr( expand_ctr, ctr_filename_ );
	SavePoints( expand_ctr, sample_filename_ );
}

void COptApp::InitCtr( Eigen::VectorXd & ctr )
{
	if ( init_ctr_file_.length() > 1 ) {
		FILE * f = fopen( init_ctr_file_.c_str(), "r" );
		if ( f != NULL ) {
			char buffer[ 1024 ];
			for ( int i = 0; i < num_ * nper_ / 3; i++ ) {
				fgets( buffer, 1024, f );
				sscanf( buffer, "%lf %lf %lf", &ctr( i * 3 ), &ctr( i * 3 + 1 ), &ctr( i * 3 + 2 ) );
			}
			fclose( f );
		}
	} else {
		for ( int l = 0; l < num_; l++ ) {
			for ( int i = 0; i <= resolution_; i++ ) {
				for ( int j = 0; j <= resolution_; j++ ) {
					for ( int k = 0; k <= resolution_; k++ ) {
						Eigen::Vector4d pos( i * unit_length_, j * unit_length_, k * unit_length_, 1 );
						Eigen::Vector4d ppos = ipose_[ l ] * pos;
						ctr( l * nper_ + GetIndex( i, j, k ) * 3 + 0 ) = ppos( 0 );
						ctr( l * nper_ + GetIndex( i, j, k ) * 3 + 1 ) = ppos( 1 );
						ctr( l * nper_ + GetIndex( i, j, k ) * 3 + 2 ) = ppos( 2 );
					}
				}
			}
		}
	}
}

void COptApp::InitCtrSLAC( Eigen::VectorXd & ctr, Eigen::VectorXd & thisCtr )
{
	for ( int i = 0; i <= resolution_; i++ ) {
		for ( int j = 0; j <= resolution_; j++ ) {
			for ( int k = 0; k <= resolution_; k++ ) {
				Eigen::Vector4d pos( i * unit_length_, j * unit_length_, k * unit_length_, 1 );
				ctr( num_ * 6 + GetIndex( i, j, k ) * 3 + 0 ) = pos( 0 );
				ctr( num_ * 6 + GetIndex( i, j, k ) * 3 + 1 ) = pos( 1 );
				ctr( num_ * 6 + GetIndex( i, j, k ) * 3 + 2 ) = pos( 2 );
			}
		}
	}
	if ( init_ctr_file_.length() > 1 ) {
		FILE * f = fopen( init_ctr_file_.c_str(), "r" );
		if ( f != NULL ) {
			char buffer[ 1024 ];
			for ( int i = 0; i < nper_ / 3; i++ ) {
				fgets( buffer, 1024, f );
				sscanf( buffer, "%lf %lf %lf", &thisCtr( num_ * 6 + i * 3 ), &thisCtr( num_ * 6 + i * 3 + 1 ), &thisCtr( num_ * 6 + i * 3 + 2 ) );
			}
			fclose( f );
		}
	} else {
		thisCtr = ctr;
	}
}


void COptApp::Pose2Ctr( std::vector< Eigen::Matrix4d > & pose, Eigen::VectorXd & ctr )
{
	for ( int l = 0; l < num_; l++ ) {
		for ( int i = 0; i <= resolution_; i++ ) {
			for ( int j = 0; j <= resolution_; j++ ) {
				for ( int k = 0; k <= resolution_; k++ ) {
					Eigen::Vector4d pos( i * unit_length_, j * unit_length_, k * unit_length_, 1 );
					Eigen::Vector4d ppos = pose[ l ] * pos;
					ctr( l * nper_ + GetIndex( i, j, k ) * 3 + 0 ) = ppos( 0 );
					ctr( l * nper_ + GetIndex( i, j, k ) * 3 + 1 ) = ppos( 1 );
					ctr( l * nper_ + GetIndex( i, j, k ) * 3 + 2 ) = ppos( 2 );
				}
			}
		}
	}
}

void COptApp::ExpandCtr( const Eigen::VectorXd & ctr, Eigen::VectorXd & expand_ctr )
{
	expand_ctr.setZero();
	for ( int l = 0; l < num_; l++ ) {
		for ( int i = 0; i < nper_ / 3; i++ ) {
			Eigen::Vector4d pos = pose_[ l ] * Eigen::Vector4d( ctr( num_ * 6 + i * 3 + 0 ), ctr( num_ * 6 + i * 3 + 1 ), ctr( num_ * 6 + i * 3 + 2 ), 1 );
			expand_ctr( l * nper_ + i * 3 + 0 ) = pos( 0 );
			expand_ctr( l * nper_ + i * 3 + 1 ) = pos( 1 );
			expand_ctr( l * nper_ + i * 3 + 2 ) = pos( 2 );
		}
	}
}

void COptApp::InitBaseAA( SparseMatrix & baseAA )
{
	TripletVector tri;

	for ( int l = 0; l < num_; l++ ) {
		HashSparseMatrix mat( l * nper_, l * nper_ );
		for ( int i = 0; i <= resolution_; i++ ) {
			for ( int j = 0; j <= resolution_; j++ ) {
				for ( int k = 0; k <= resolution_; k++ ) {
					int idx[ 2 ] = { GetIndex( i, j, k ) * 3, 0 };
					double val[ 2 ] = { 1, -1 };
					if ( i > 0 ) {
						idx[ 1 ] = GetIndex( i - 1, j, k ) * 3;
						mat.AddHessian2( idx, val, tri );
					}
					if ( i < resolution_ ) {
						idx[ 1 ] = GetIndex( i + 1, j, k ) * 3;
						mat.AddHessian2( idx, val, tri );
					}
					if ( j > 0 ) {
						idx[ 1 ] = GetIndex( i, j - 1, k ) * 3;
						mat.AddHessian2( idx, val, tri );
					}
					if ( j < resolution_ ) {
						idx[ 1 ] = GetIndex( i, j + 1, k ) * 3;
						mat.AddHessian2( idx, val, tri );
					}
					if ( k > 0 ) {
						idx[ 1 ] = GetIndex( i, j, k - 1 ) * 3;
						mat.AddHessian2( idx, val, tri );
					}
					if ( k < resolution_ ) {
						idx[ 1 ] = GetIndex( i, j, k + 1 ) * 3;
						mat.AddHessian2( idx, val, tri );
					}
				}
			}
		}
		if ( l == 0 ) {
			mat.Add( 0, 0, 1, tri );
			mat.Add( 1, 1, 1, tri );
			mat.Add( 2, 2, 1, tri );
		}
	}

	baseAA.setFromTriplets( tri.begin(), tri.end() );
}

void COptApp::InitBaseJJ( SparseMatrix & baseJJ )
{
	TripletVector tri;

	HashSparseMatrix mat( 6 * num_, 6 * num_ );
	for ( int i = 0; i <= resolution_; i++ ) {
		for ( int j = 0; j <= resolution_; j++ ) {
			for ( int k = 0; k <= resolution_; k++ ) {
				int idx[ 2 ] = { GetIndex( i, j, k ) * 3, 0 };
				double val[ 2 ] = { 1, -1 };
				if ( i > 0 ) {
					idx[ 1 ] = GetIndex( i - 1, j, k ) * 3;
					mat.AddHessian2( idx, val, tri );
				}
				if ( i < resolution_ ) {
					idx[ 1 ] = GetIndex( i + 1, j, k ) * 3;
					mat.AddHessian2( idx, val, tri );
				}
				if ( j > 0 ) {
					idx[ 1 ] = GetIndex( i, j - 1, k ) * 3;
					mat.AddHessian2( idx, val, tri );
				}
				if ( j < resolution_ ) {
					idx[ 1 ] = GetIndex( i, j + 1, k ) * 3;
					mat.AddHessian2( idx, val, tri );
				}
				if ( k > 0 ) {
					idx[ 1 ] = GetIndex( i, j, k - 1 ) * 3;
					mat.AddHessian2( idx, val, tri );
				}
				if ( k < resolution_ ) {
					idx[ 1 ] = GetIndex( i, j, k + 1 ) * 3;
					mat.AddHessian2( idx, val, tri );
				}
			}
		}
	}

	int base_anchor = GetIndex( resolution_ / 2, resolution_ / 2, 0 ) * 3;

	mat.Add( base_anchor + 0, base_anchor + 0, 1, tri );
	mat.Add( base_anchor + 1, base_anchor + 1, 1, tri );
	mat.Add( base_anchor + 2, base_anchor + 2, 1, tri );

	baseJJ.setFromTriplets( tri.begin(), tri.end() );
}

Eigen::Matrix3d COptApp::GetRotation( const int idx, const std::vector< int > & idxx, const Eigen::VectorXd & ictr, const Eigen::VectorXd & ctr )
{
	int n = ( int )idxx.size();
	Eigen::Matrix3d C = Eigen::Matrix3d::Zero();

	for ( int i = 0; i < n; i++ ) {
		Eigen::RowVector3d dif( ictr( idx ) - ictr( idxx[ i ] ), ictr( idx + 1 ) - ictr( idxx[ i ] + 1 ), ictr( idx + 2 ) - ictr( idxx[ i ] + 2 ) );
		Eigen::RowVector3d diff( ctr( idx ) - ctr( idxx[ i ] ), ctr( idx + 1 ) - ctr( idxx[ i ] + 1 ), ctr( idx + 2 ) - ctr( idxx[ i ] + 2 ) );
		C += dif.transpose() * diff;
	}

	Eigen::JacobiSVD< Eigen::Matrix3d > svd( C, Eigen::ComputeFullU | Eigen::ComputeFullV );
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	Eigen::Matrix3d R = V * U.transpose();
	if ( R.determinant() < 0 ) {
		U( 0, 2 ) *= -1;
		U( 1, 2 ) *= -1;
		U( 2, 2 ) *= -1;
		R = V * U.transpose();
	}
	return R;
}

void COptApp::SaveCtr( const Eigen::VectorXd & ctr, std::string filename )
{
	PCL_INFO( "Save ctr to file %s ... ", filename.c_str() );
	FILE * f = fopen( filename.c_str(), "w" );
	if ( f != NULL ) {
		for ( int i = 0; i < num_ * nper_ / 3; i++ ) {
			fprintf( f, "%.10f %.10f %.10f\n", ctr( i * 3 ), ctr( i * 3 + 1 ), ctr( i * 3 + 2 ) );
		}
		fclose( f );
	}
	PCL_INFO( "Done.\n" );
}

void COptApp::SavePoints( const Eigen::VectorXd & ctr, std::string filename )
{
	if ( sample_num_ <= 0 )
		return;

	pcl::PointCloud< pcl::PointXYZRGBNormal > pcd;
	pcl::PointXYZRGBNormal pt;
	for ( int l = 0; l < num_; l++ ) {
		pointclouds_[ l ].UpdateAllNormal( ctr );
		for ( int i = 0; i < ( int )pointclouds_[ l ].points_.size(); i += sample_num_ ) {
			Eigen::Vector3d pos = pointclouds_[ l ].UpdatePoint( ctr, pointclouds_[ l ].points_[ i ] );
			Eigen::Vector3d norm( pointclouds_[ l ].points_[ i ].n_[ 0 ], pointclouds_[ l ].points_[ i ].n_[ 1 ], pointclouds_[ l ].points_[ i ].n_[ 2 ] );
			norm.normalize();
			pt.x = pos( 0 );
			pt.y = pos( 1 );
			pt.z = pos( 2 );
			pt.normal_x = norm( 0 );
			pt.normal_y = norm( 1 );
			pt.normal_z = norm( 2 );
			pcd.push_back( pt );
		}
	}

	PCL_INFO( "Save sample pcd into %s ... ", filename.c_str() );
	pcl::PCDWriter w;
	w.writeBinaryCompressed( filename, pcd );
}

void COptApp::Blacklist( std::string filename )
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
