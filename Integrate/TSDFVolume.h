#pragma once
#include <unordered_set>
#include <unordered_map>

#include "TSDFVolumeUnit.h"

/////////////////////////////////////////////////////
// hash function of TSDFVolumeUnit
// -256 ~ 256 on each axis
// indexing of the volumes
// x * 64 ~ x * 64 + 63
/////////////////////////////////////////////////////

class TSDFVolume
{
public:
	TSDFVolume( int cols, int rows );
	~TSDFVolume(void);

public:
	CameraParam camera_;
	int cols_, rows_;

	const double unit_length_;
	const double tsdf_trunc_;

	std::unordered_map< int, TSDFVolumeUnit::Ptr > data_;

public:
	void ScaleDepth( std::vector< unsigned short > & depth, std::vector< float > & scaled );
	void Integrate( std::vector< unsigned short > & depth, std::vector< float > & scaled, Eigen::Matrix4d & transformation );
	void IntegrateVolumeUnit( std::vector< float > & scaled, const Eigen::Matrix4f & trans, const Eigen::Matrix4f & trans_inv, TSDFVolumeUnit::Ptr volume, float x_shift, float y_shift, float z_shift );
	void SaveWorld( std::string filename );

public:
	int round( double x ) {
		return static_cast< int >( floor( x + 0.5 ) );
	}

	bool UVD2XYZ( int u, int v, unsigned short d, double & x, double & y, double & z ) {
		if ( d > 0 ) {
			z = d / 1000.0;
			x = ( u - camera_.cx_ ) * z / camera_.fx_;
			y = ( v - camera_.cy_ ) * z / camera_.fy_;
			return true;
		} else {
			return false;
		}
	}

	bool XYZ2UVD( double x, double y, double z, int & u, int & v, unsigned short & d ) {
		if ( z > 0 ) {
			u = round( x * camera_.fx_ / z + camera_.cx_ );
			v = round( y * camera_.fy_ / z + camera_.cy_ );
			d = static_cast< unsigned short >( round( z * 1000.0 ) );
			return ( u >= 0 && u < 640 && v >= 0 && v < 480 );
		} else {
			return false;
		}
	}

	int hash_key( int x, int y, int z ) {
		return x * 512 * 512 + y * 512 + z;
	}

	float I2F( int i ) {
		return ( ( i - 256 ) * 64 * unit_length_ );
	}
};

