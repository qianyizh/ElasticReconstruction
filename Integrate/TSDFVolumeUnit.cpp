#include "StdAfx.h"
#include "TSDFVolumeUnit.h"

TSDFVolumeUnit::TSDFVolumeUnit(void)
	: resolution_( 64 )
{
	int vol_size = resolution_ * resolution_ * resolution_;

	sdf_ = new float[ vol_size ];
	memset( sdf_, 0, sizeof( *sdf_ ) * vol_size );

	weight_ = new float[ vol_size ];
	memset( weight_, 0, sizeof( *weight_ ) * vol_size );
}

TSDFVolumeUnit::~TSDFVolumeUnit(void)
{
	delete []sdf_;
	delete []weight_;
}
