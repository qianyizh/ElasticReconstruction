#pragma once

#include <vector>
#include <hash_set>
#include "external/Eigen/Core"
#include "external/Eigen/Geometry"
#include "external/Eigen/SparseCore"
#include "HashSparseMatrix.h"
#include "PointCloud.h"
#include "RGBDTrajectory.h"

typedef Eigen::SparseMatrix< double > SparseMatrix;

typedef std::pair< int, int > CorrespondencePair;

struct Correspondence {
public:
	int idx0_, idx1_;
	Eigen::Matrix4d trans_;
	std::vector< CorrespondencePair > corres_;
public:
	Correspondence( int i0, int i1 ) : idx0_( i0 ), idx1_( i1 ) {}
	void LoadFromFile( std::string filename ) {
		FILE * f = fopen( filename.c_str(), "r" );
		if ( f != NULL ) {
			char buffer[1024];
			CorrespondencePair pair;
			while ( fgets( buffer, 1024, f ) != NULL ) {
				if ( strlen( buffer ) > 0 && buffer[ 0 ] != '#' ) {
					sscanf( buffer, "%d %d", &pair.first, &pair.second );
					corres_.push_back( pair );
				}
			}
			fclose ( f );
		}
	}
};

class COptApp
{
public:
	COptApp(void);
	~COptApp(void);

public:
	RGBDTrajectory rgbd_traj_;
	RGBDTrajectory reg_traj_;
	int resolution_;
	int interval_;
	int num_;
	double weight_;
	double length_;
	int max_iteration_;
	int max_inner_iteration_;
	std::string dir_prefix_;
	std::string rgbd_filename_;
	std::string reg_filename_;
	std::string ctr_filename_;
	std::string sample_filename_;
	std::string init_ctr_file_;
	std::string pose_filename_;
	int sample_num_;
	int blacklist_pair_num_;

	// hidden parameters
	double unit_length_;
	int nper_;
	std::vector< Eigen::Matrix4d > ipose_;
	int matrix_size_;
	
	// global point cloud, let's trust the memory size ...
	std::vector< PointCloud > pointclouds_;

	// global correspondences
	std::vector< Correspondence > corres_;

	// black listed fragments
	stdext::hash_set< int > blacklist_;
	std::vector< int > absolute2relative_map_;
	std::vector< int > relative2absolute_map_;

	// rigid
	std::vector< Eigen::Matrix4d > pose_;
	RGBDTrajectory output_traj_;

	// slac
	std::vector< Eigen::Matrix3d > pose_rot_t_;

public:
	void Blacklist( std::string filename );
	void IPoseFromFile( std::string filename ) {
		RGBDTrajectory ip;
		ip.LoadFromFile( filename );
		ipose_.resize( ip.data_.size() );
		for ( int i = 0; i < ( int )ipose_.size(); i++ ) {
			ipose_[ i ] = ip.data_[ i ].transformation_;
		}
	}

public:
	void OptimizeNonrigid();
	void OptimizeRigid();
	void OptimizeSLAC();

private:
	void InitMap();
	void InitIPose();
	void InitPointClouds();
	void InitCorrespondences();
	void InitCtr( Eigen::VectorXd & ctr );
	void InitCtrSLAC( Eigen::VectorXd & ctr, Eigen::VectorXd & thisCtr );

	void Pose2Ctr( std::vector< Eigen::Matrix4d > & pose, Eigen::VectorXd & ctr );
	void InitBaseAA( SparseMatrix & baseAA );
	void InitBaseJJ( SparseMatrix & baseJJ );
	void SaveCtr( const Eigen::VectorXd & ctr, std::string filename );
	void SavePoints( const Eigen::VectorXd & ctr, std::string filename );
	inline int GetIndex( int i, int j, int k ) {
		return i + j * ( resolution_ + 1 ) + k * ( resolution_ + 1 ) * ( resolution_ + 1 );
	}
	Eigen::Matrix3d GetRotation( const int idx, const std::vector< int > & idxx, const Eigen::VectorXd & ictr, const Eigen::VectorXd & ctr );
	void ExpandCtr( const Eigen::VectorXd & ctr, Eigen::VectorXd & expand_ctr );
};

