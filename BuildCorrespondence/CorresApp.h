#pragma once

#include <vector>
#include <fstream>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <hash_set>
#include <hash_map>

typedef pair< int, int > IntPair;

struct FramedTransformation {
	int id1_;
	int id2_;
	int frame_;
	Eigen::Matrix4d transformation_;
	FramedTransformation( int id1, int id2, int f, Eigen::Matrix4d t )
		: id1_( id1 ), id2_( id2 ), frame_( f ), transformation_( t ) 
	{}
};

struct RGBDTrajectory {
	std::vector< FramedTransformation > data_;
	int index_;

	void LoadFromFile( std::string filename ) {
		data_.clear();
		index_ = 0;
		int id1, id2, frame;
		Eigen::Matrix4d trans;
		FILE * f = fopen( filename.c_str(), "r" );
		if ( f != NULL ) {
			char buffer[1024];
			while ( fgets( buffer, 1024, f ) != NULL ) {
				if ( strlen( buffer ) > 0 && buffer[ 0 ] != '#' ) {
					sscanf( buffer, "%d %d %d", &id1, &id2, &frame);
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(0,0), &trans(0,1), &trans(0,2), &trans(0,3) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(1,0), &trans(1,1), &trans(1,2), &trans(1,3) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(2,0), &trans(2,1), &trans(2,2), &trans(2,3) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(3,0), &trans(3,1), &trans(3,2), &trans(3,3) );
					data_.push_back( FramedTransformation( id1, id2, frame, trans ) );
				}
			}
			fclose( f );
		}
	}
	void SaveToFile( std::string filename ) {
		FILE * f = fopen( filename.c_str(), "w" );
		for ( int i = 0; i < ( int )data_.size(); i++ ) {
			Eigen::Matrix4d & trans = data_[ i ].transformation_;
			fprintf( f, "%d\t%d\t%d\n", data_[ i ].id1_, data_[ i ].id2_, data_[ i ].frame_ );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(0,0), trans(0,1), trans(0,2), trans(0,3) );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(1,0), trans(1,1), trans(1,2), trans(1,3) );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(2,0), trans(2,1), trans(2,2), trans(2,3) );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(3,0), trans(3,1), trans(3,2), trans(3,3) );
		}
		fclose( f );
	}
};

typedef std::pair< int, int > CorrespondencePair;

class CCorresApp
{
public:
	CCorresApp(void);
	~CCorresApp(void);

public:
	RGBDTrajectory corres_traj_;
	double dist_thresh_;
	double normal_thresh_;
	bool save_xyzn_;
	bool save_corres_;

	double reg_dist_;
	double reg_ratio_;
	double bbox_length_;
	int reg_num_;
	bool registration_;

	std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > pointclouds_;

	char m_pDirName[ 1024 ];

	stdext::hash_set< int > blacklist_;

	bool redux_;
	RGBDTrajectory redux_traj_;
	stdext::hash_map< int, int > redux_map_;
	int num_;
	int interval_;
	double length_;

public:
	void LoadData( std::string filename, int num );
	void FindCorrespondence();
	void Registration();
	void Finalize();
	void Blacklist( std::string filename );
	void Redux( std::string filename );

public:
	void ToggleSaveXYZN() {
		save_xyzn_ = true;
	}
	double NormalDot( pcl::PointXYZRGBNormal & p0, pcl::PointXYZRGBNormal & p1 ) {
		return ( p0.normal_x * p1.normal_x + p0.normal_y * p1.normal_y + p0.normal_z * p1.normal_z );
	}
	int GetReduxIndex( int i, int j ) {
		return i + j * num_;
	}
	double GetVolumeOverlapRatio( Eigen::Matrix4d & trans ) {
		int res = 20;
		double ul = length_ / ( double )res;
		int s = 0;
		for ( int i = 0; i < res; i++ ) {
			for ( int j = 0; j < res; j++ ) {
				for ( int k = 0; k < res; k++ ) {
					Eigen::Vector4d pos( ( i + 0.5 ) * ul, ( j + 0.5 ) * ul, ( k + 0.5 ) * ul, 1 );
					Eigen::Vector4d ppos = trans * pos;
					if ( ppos( 0 ) >= 0 && ppos( 0 ) <= length_ && ppos( 1 ) >= 0 && ppos( 1 ) <= length_ && ppos( 2 ) >= 0 && ppos( 2 ) <= length_ ) {
						s++;
					}
				}
			}
		}

		return ( double )s / res / res / res;
	}
};
