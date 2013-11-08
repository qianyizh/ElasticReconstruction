#include "StdAfx.h"
#include "OptApp.h"
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>

COptApp::COptApp(void)
	: resolution_( 8 )
	, interval_( 50 )
	, num_( 0 )
	, weight_( 1.0f )
	, length_( 3.0f )
	, max_iteration_( 5 )
	, max_inner_iteration_( 10 )
	, max_pcg_iteration_( 100 )
	, conv_score_( 0.01 )
	, ctr_filename_( "output.ctr" )
	, sample_filename_( "sample.pcd" )
	, init_ctr_file_( "" )
	, dir_prefix_( "" )
	, sample_num_( -1 )
	, blacklist_pair_num_( 10000 )
{
}

COptApp::~COptApp(void)
{
}

void COptApp::Init()
{
	InitMap();
	unit_length_ = length_ / resolution_;
	nper_ = ( resolution_ + 1 ) * ( resolution_ + 1 ) * ( resolution_ + 1 ) * 3;
	matrix_size_ = nper_ * num_;
	InitIPose();
	InitPointClouds();
	InitCorrespondences();
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
		sprintf( filename, "%scloud_bin_xyzn_%d.xyzn", dir_prefix_.c_str(), ii );
		pointclouds_[ i ].LoadFromFile( filename );
	}
	if ( sample_num_ > 0 ) {
		int sum_to_report = 0;
		cout << "Sample number sections" << endl;
		for ( int i = 0; i < num_; i++ ) {
			int this_num = pointclouds_[ i ].points_.size() / sample_num_;
			cout << "Point cloud #" << i << ": " << sum_to_report << " - " << sum_to_report + this_num << endl;
			sum_to_report += this_num;
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
			//cout << filename << " - " << reg_traj_.data_[ i ].frame_ << " : " << corres_.back().corres_.size() << endl;
		}
	}
}

void COptApp::Optimize()
{
	PCL_WARN( "Optimization with weight %.5f, resolution %d, piece number %d, max iteration %d\n", weight_, resolution_, num_, max_iteration_ );

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
			//cout << pointclouds_[ l ].points_[ 99 ].n_[ 0 ] << ", " << pointclouds_[ l ].points_[ 99 ].n_[ 1 ] << ", " << pointclouds_[ l ].points_[ 99 ].n_[ 2 ] << endl;
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

			//cout << l << ": " << i << ", " << j << "in" << endl;

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
			/*
			if ( i == 0 && j == 1 ) {
				for ( int iii = 1100; iii < 1200; iii++ ) {
					cout << tri[ iii ].m_row << ", " << tri[ iii ].m_col << ": " << tri[ iii ].m_value << endl;
				}
			}
			*/

			SparseMatrix tempAA( matrix_size_, matrix_size_ );
			tempAA.setFromTriplets( tri.begin(), tri.end() );

			#pragma omp critical
			{
				nprocessed++;
				thisAA += tempAA;
				//cout << l << ": " << i << ", " << j << "out" << endl;
				//PCL_INFO( "%d / %d processed\n", nprocessed, corres_.size() );
				PCL_INFO( "\b\b\b\b\b%5d", nprocessed );
			}
		}
		PCL_INFO( " ... Done.\n" );

		//Eigen::saveMarket(thisAA, "thisAA.mtx");

		Eigen::CholmodSupernodalLLT< SparseMatrix, Eigen::Upper > solver;
		//Eigen::ConjugateGradient< SparseMatrix, Eigen::Upper > solver;
		solver.analyzePattern( thisAA );
		solver.factorize( thisAA );
		//solver.setMaxIterations( max_pcg_iteration_ );

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
			//PCL_INFO( "b estimated, now solve linear system.\n" );
		
			oldctr = ctr;

			/*
			Eigen::ConjugateGradient< SparseMatrix, Eigen::Upper > solver;
			solver.setMaxIterations( max_pcg_iteration_ );
			ctr = solver.compute( thisAA ).solveWithGuess( Ab, oldctr );
			*/
			//ctr = solver.compute( thisAA ).solve( Ab );
			//Eigen::CholmodSupernodalLLT< SparseMatrix, Eigen::Upper > solver;
			//ctr = solver.compute( thisAA ).solve( Ab );

			ctr = solver.solve( Ab );
			//ctr = solver.solveWithGuess( Ab, oldctr );

			score = ( oldctr - ctr ).norm();
			PCL_WARN( "Iteration #%d:%d (%d:%d) : score is %.4f\n", itr + 1, m + 1, max_iteration_, max_inner_iteration_, score );

			if ( sample_num_ > 0 ) {
				char filename[ 1024 ];
				memset( filename, 0, 1024 );
				sprintf( filename, "itr%d_inner%d_out.ctr", itr, m );
				SaveCtr( ctr, filename );
			}


			if ( score < conv_score_ ) {
				break;
			}

			/*
			for ( int kkk = 0; kkk < 20; kkk++ ) {
				cout << Ab( kkk ) << endl;
			}
			cout << endl;

			for ( int kkk = 0; kkk < 20; kkk++ ) {
				cout << ctr( kkk ) << endl;
			}
			*/
		}

		if ( score < conv_score_ ) {
			break;
		}
	}

	SaveCtr( ctr, ctr_filename_ );
	SavePoints( ctr, sample_filename_ );
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

	/*
	for ( int i = 0; i < 100; i++ ) {
		cout << tri[ i ].m_row << ", " << tri[ i ].m_col << ": " << tri[ i ].m_value << endl;
	}
	*/

	baseAA.setFromTriplets( tri.begin(), tri.end() );
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
