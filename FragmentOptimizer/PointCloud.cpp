#include "StdAfx.h"
#include "PointCloud.h"


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

void PointCloud::LoadFromFile( const char * filename )
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
					printf( "Error!!\n" );
					return;
				}
			}
		}
		fclose ( f );
		printf( "Read %s ... get %d points.\n", filename, points_.size() );
	}
}