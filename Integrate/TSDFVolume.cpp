#include "StdAfx.h"
#include "TSDFVolume.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

TSDFVolume::TSDFVolume( int cols, int rows )
	: cols_( cols )
	, rows_( rows )
	, unit_length_( 3.0 / 512.0 )
	, tsdf_trunc_( 0.03 )
{
}

TSDFVolume::~TSDFVolume(void)
{
}

void TSDFVolume::ScaleDepth( std::vector< unsigned short > & depth, std::vector< float > & scaled )
{
#pragma omp parallel for num_threads( 8 )
	for ( int y = 0; y < rows_; y++ ) {
		for ( int x = 0; x < cols_; x++ ) {
			unsigned short d = depth[ y * cols_ + x ];
			float xl = ( x - camera_.cx_ ) / camera_.fx_;
			float yl = ( y - camera_.cy_ ) / camera_.fy_;
			float lambda = sqrtf( xl * xl + yl * yl + 1 );
			float res = d * lambda / 1000.f;
			if ( res > camera_.integration_trunc_ ) {
				scaled[ y * cols_ + x ] = 0.0f;
			} else {
				scaled[ y * cols_ + x ] = d * lambda / 1000.f;
			}
		}
	}
}

void TSDFVolume::Integrate( std::vector< unsigned short > & depth, std::vector< float > & scaled, Eigen::Matrix4d & transformation )
{
	Eigen::Matrix4d trans_inv = transformation.inverse();
	std::unordered_set< int > touched_unit;

	double x, y, z;
	int xi, yi, zi, key;
	for ( int v = 0; v < rows_; v += 1 ) {
		for ( int u = 0; u < cols_; u += 1 ) {
			unsigned short d = depth[ v * cols_ + u ];
			if ( UVD2XYZ( u, v, d, x, y, z ) ) {
				Eigen::Vector4d vv = transformation * Eigen::Vector4d( x, y, z, 1 );
				xi = ( ( int )floor( vv( 0 ) / unit_length_ + 0.5 ) + ( 256 * 64 ) ) / 64;
				yi = ( ( int )floor( vv( 1 ) / unit_length_ + 0.5 ) + ( 256 * 64 ) ) / 64;
				zi = ( ( int )floor( vv( 2 ) / unit_length_ + 0.5 ) + ( 256 * 64 ) ) / 64;
				key = hash_key( xi, yi, zi );
				if ( touched_unit.find( key ) == touched_unit.end() ) {
					touched_unit.insert( key );
					if ( data_.find( key ) == data_.end() ) {
						data_[ key ] = TSDFVolumeUnit::Ptr( new TSDFVolumeUnit( 64, xi, yi, zi ) );
					}
					IntegrateVolumeUnit( scaled, transformation.cast< float >(), trans_inv.cast< float >(), data_[ key ], I2F( xi ), I2F( yi ), I2F( zi ) );
				}
			}
		}
	}
	//for ( std::unordered_map< int, TSDFVolumeUnit::Ptr >::iterator it = data_.begin(); it != data_.end(); it++ ) {
	//	IntegrateVolumeUnit( scaled,  transformation.cast< float >(), trans_inv.cast< float >(), it->second, I2F( it->second->xi_ ), I2F( it->second->yi_ ), I2F( it->second->zi_ ) );
	//}
}

void TSDFVolume::IntegrateVolumeUnit( std::vector< float > & scaled, const Eigen::Matrix4f & trans, const Eigen::Matrix4f & trans_inv, TSDFVolumeUnit::Ptr volume, float x_shift, float y_shift, float z_shift )
{
#pragma omp parallel for num_threads( 8 )
	for ( int i = 0; i < volume->resolution_; i++ ) {
		for ( int j = 0; j < volume->resolution_; j++ ) {
			for ( int k = 0; k < volume->resolution_; k++ ) {
				Eigen::Vector4f gridv( i * unit_length_ + x_shift, j * unit_length_ + y_shift, k * unit_length_ + z_shift, 1 );
				Eigen::Vector4f tgv = trans_inv * gridv;
				if ( tgv( 2 ) > 0  ) {
					int coox = round( tgv( 0 ) * camera_.fx_ / tgv( 2 ) + camera_.cx_ );
					int cooy = round( tgv( 1 ) * camera_.fy_ / tgv( 2 ) + camera_.cy_ );
					if ( coox >= 0 && coox < cols_ && cooy >= 0 && cooy < rows_ ) {
						float dp_scaled = scaled[ cooy * cols_ + coox ];
						if ( dp_scaled > 0.001f ) {
							float rx = gridv( 0 ) - trans( 0, 3 );
							float ry = gridv( 1 ) - trans( 1, 3 );
							float rz = gridv( 2 ) - trans( 2, 3 );
							float sdf = dp_scaled - sqrtf( rx * rx + ry * ry + rz * rz );
							if ( sdf >= - tsdf_trunc_ ) {
								float tsdf = std::min< float >( 1.0f, sdf / tsdf_trunc_ );
								// need to change weight
								float w = 1.0f;

								int lll = ( i * volume->resolution_ + j ) * volume->resolution_ + k;
								volume->sdf_[ lll ] = ( volume->sdf_[ lll ] * volume->weight_[ lll ] + w * tsdf ) / ( volume->weight_[ lll ] + w );
								volume->weight_[ lll ] += w;
							}
						}
					}
				}			
			}
		}
	}
}

void TSDFVolume::SaveWorld( std::string filename )
{
	using namespace pcl;
	PointCloud< PointXYZI >::Ptr world( new PointCloud< PointXYZI > );

	PointXYZI p;
	for ( std::unordered_map< int, TSDFVolumeUnit::Ptr >::iterator it = data_.begin(); it != data_.end(); it++ ) {
		TSDFVolumeUnit::Ptr unit = it->second;
		float * sdf = unit->sdf_;
		float * w = unit->weight_;

		for ( int i = 0; i < unit->resolution_; i++ ) {
			for ( int j = 0; j < unit->resolution_; j++ ) {
				for ( int k = 0; k < unit->resolution_; k++, sdf++, w++ ) {
					if ( *w != 0.0f && *sdf < 0.98f && *sdf >= -0.98f ) {
						p.x = i + ( unit->xi_ - 256 ) * 64;
						p.y = j + ( unit->yi_ - 256 ) * 64;
						p.z = k + ( unit->zi_ - 256 ) * 64;
						p.intensity = *sdf;
						world->push_back( p );
					}
				}
			}
		}
	}

	pcl::io::savePCDFile( filename, *world, true );
	PCL_INFO( "%d voxel points have been written.\n", world->size() );
}