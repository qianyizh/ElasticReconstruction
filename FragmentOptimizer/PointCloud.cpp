#include "StdAfx.h"
#include "PointCloud.h"
#include <pcl/io/pcd_io.h>


PointCloud::PointCloud( int index, int resolution, float length )
{
	resolution_ = resolution;
	length_ = length;
	unit_length_ = length / resolution;
	index_ = index;
	nper_ = ( resolution_ + 1 ) * ( resolution_ + 1 ) * ( resolution_ + 1 ) * 3;
	offset_ = index * nper_;
}

PointCloud::~PointCloud(void)
{
}

void PointCloud::LoadFromPCDFile( const char * filename )
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rawpcd( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
	if ( pcl::io::loadPCDFile( filename, *rawpcd ) < 0 ) {
		PCL_ERROR( "Error loading file.\n" );
		return;
	}
	float x[ 6 ];
	for ( int i = 0; i < ( int )rawpcd->points.size(); i++ ) {
		if ( !_isnan( rawpcd->points[ i ].normal_x ) ) {
			points_.resize( points_.size() + 1 );
			x[ 0 ] = rawpcd->points[ i ].x;
			x[ 1 ] = rawpcd->points[ i ].y;
			x[ 2 ] = rawpcd->points[ i ].z;
			x[ 3 ] = rawpcd->points[ i ].normal_x;
			x[ 4 ] = rawpcd->points[ i ].normal_y;
			x[ 5 ] = rawpcd->points[ i ].normal_z;
			if ( GetCoordinate( x, points_.back() ) == false ) {
				PCL_ERROR( "Error!! Point out of bound!!\n" );
				return;
			}
		}
	}
	printf( "Read %s ... get %d points.\n", filename, points_.size() );
}

void PointCloud::LoadFromXYZNFile( const char * filename )
{
	FILE * f = fopen( filename, "r" );
	if ( f != NULL ) {
		char buffer[1024];
		float x[ 6 ];
		while ( fgets( buffer, 1024, f ) != NULL ) {
			if ( strlen( buffer ) > 0 && buffer[ 0 ] != '#' ) {
				sscanf( buffer, "%f %f %f %f %f %f", &x[ 0 ], &x[ 1 ], &x[ 2 ], &x[ 3 ], &x[ 4 ], &x[ 5 ] );
				points_.resize( points_.size() + 1 );
				if ( GetCoordinate( x, points_.back() ) == false ) {
					PCL_ERROR( "Error!! Point out of bound!!\n" );
					return;
				}
			}
		}
		fclose ( f );
		printf( "Read %s ... get %d points.\n", filename, points_.size() );
	}
}