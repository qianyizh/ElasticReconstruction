#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <hash_set>
#include <hash_map>

#include "Helper.h"

typedef std::pair< int, int > CorrespondencePair;

class CCorresApp
{
public:
	CCorresApp(void);
	~CCorresApp(void);

public:
	RGBDTrajectory corres_traj_;
	RGBDInformation corres_info_;
	double dist_thresh_;
	double normal_thresh_;
	bool save_xyzn_;
	bool save_corres_;

	double reg_dist_;
	double reg_ratio_;
	double bbox_length_;
	int reg_num_;
	bool registration_;
	bool output_information_;

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
