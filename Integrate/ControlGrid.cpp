#include "StdAfx.h"
#include "ControlGrid.h"


ControlGrid::ControlGrid(void)
	: resolution_( 8 )
{
}


ControlGrid::~ControlGrid(void)
{
}

void ControlGrid::Load( FILE * f, int res, float len )
{
	resolution_ = res;
	length_ = len;
	unit_length_ = length_ / resolution_;

	int total = ( res + 1 ) * ( res + 1 ) * ( res + 1 );
	ctr_.resize( total );
	for ( int i = 0; i < total; i++ ) {
		char buffer[1024];
		if ( fgets( buffer, 1024, f ) != NULL ) {
			if ( strlen( buffer ) > 0 && buffer[ 0 ] != '#' ) {
				sscanf( buffer, "%f %f %f", &ctr_[ i ]( 0 ), &ctr_[ i ]( 1 ), &ctr_[ i ]( 2 ));
				RegulateBBox( ctr_[ i ]( 0 ), 0 );
				RegulateBBox( ctr_[ i ]( 1 ), 1 );
				RegulateBBox( ctr_[ i ]( 2 ), 2 );
			}
		}
	}
}