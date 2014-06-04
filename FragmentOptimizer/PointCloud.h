#pragma once

#include <vector>
#include "external/Eigen/Core"

struct Point {
public:
	int idx_[ 8 ];
	float n_[ 3 ];
	float val_[ 8 ];
	float nval_[ 8 ];
	float p_[ 3 ];
};

class PointCloud
{
public:
	PointCloud( int index = 0, int resolution = 8, float length = 3.0f );
	~PointCloud(void);

public:
	int resolution_;
	int nper_;
	int offset_;
	int index_;
	float length_;
	float unit_length_;

public:
	std::vector< Point > points_;

public:
	void LoadFromPCDFile( const char * filename );
	void LoadFromXYZNFile( const char * filename );

public:
	void UpdateAllNormal( const Eigen::VectorXd & ctr ) {
		for ( int i = 0; i < ( int )points_.size(); i++ ) {
			UpdateNormal( ctr, points_[ i ] );
		}
	}

	void UpdateAllPointPN( const Eigen::VectorXd & ctr ) {
		for ( int i = 0; i < ( int )points_.size(); i++ ) {
			UpdateNormal( ctr, points_[ i ] );
			Eigen::Vector3d pos = UpdatePoint( ctr, points_[ i ] );
			points_[ i ].p_[ 0 ] = pos( 0 );
			points_[ i ].p_[ 1 ] = pos( 1 );
			points_[ i ].p_[ 2 ] = pos( 2 );
		}
	}

	inline int GetIndex( int i, int j, int k ) {
		return i + j * ( resolution_ + 1 ) + k * ( resolution_ + 1 ) * ( resolution_ + 1 );
	}

	inline void UpdateNormal( const Eigen::VectorXd & ctr, Point & point ) {
		for ( int i = 0; i < 3; i++ ) {
			point.n_[ i ] = 0.0f;
			for ( int j = 0; j < 8; j++ ) {
				point.n_[ i ] += point.nval_[ j ] * ( float )ctr( point.idx_[ j ] + i + offset_ );
			}
		}
		float len = sqrt( point.n_[ 0 ] * point.n_[ 0 ] + point.n_[ 1 ] * point.n_[ 1 ] + point.n_[ 2 ] * point.n_[ 2 ] );
		point.n_[ 0 ] /= len;
		point.n_[ 1 ] /= len;
		point.n_[ 2 ] /= len;
	}

	inline void UpdatePose( const Eigen::Matrix4f & inc_pose ) {
		Eigen::Vector4f p, n;
		for ( int i = 0; i < ( int )points_.size(); i++ ) {
			Point & point = points_[ i ];
			p = inc_pose * Eigen::Vector4f( point.p_[ 0 ], point.p_[ 1 ], point.p_[ 2 ], 1 );
			n = inc_pose * Eigen::Vector4f( point.n_[ 0 ], point.n_[ 1 ], point.n_[ 2 ], 0 );
			point.p_[ 0 ] = p( 0 );
			point.p_[ 1 ] = p( 1 );
			point.p_[ 2 ] = p( 2 );
			point.n_[ 0 ] = n( 0 );
			point.n_[ 1 ] = n( 1 );
			point.n_[ 2 ] = n( 2 );
		}
	}

	inline Eigen::Vector3d UpdatePoint( const Eigen::VectorXd & ctr, Point & point ) {
		Eigen::Vector3d pos;
		for ( int i = 0; i < 3; i++ ) {
			pos( i ) = 0.0;
			for ( int j = 0; j < 8; j++ ) {
				pos( i ) += point.val_[ j ] * ( float )ctr( point.idx_[ j ] + i + offset_ );
			}
		}
		return pos;
	}

	inline bool GetCoordinate( float pt[ 6 ], Point & point ) {
		point.p_[ 0 ] = pt[ 0 ];
		point.p_[ 1 ] = pt[ 1 ];
		point.p_[ 2 ] = pt[ 2 ];

		int corner[ 3 ] = {
			( int )floor( pt[ 0 ] / unit_length_ ),
			( int )floor( pt[ 1 ] / unit_length_ ),
			( int )floor( pt[ 2 ] / unit_length_ )
		};

		if ( corner[ 0 ] < 0 || corner[ 0 ] >= resolution_
			|| corner[ 1 ] < 0 || corner[ 1 ] >= resolution_
			|| corner[ 2 ] < 0 || corner[ 2 ] >= resolution_ )
			return false;

		float residual[ 3 ] = {
			pt[ 0 ] / unit_length_ - corner[ 0 ],
			pt[ 1 ] / unit_length_ - corner[ 1 ],
			pt[ 2 ] / unit_length_ - corner[ 2 ]
		};
		// for speed, skip sanity check
		point.idx_[ 0 ] = GetIndex( corner[ 0 ], corner[ 1 ], corner[ 2 ] ) * 3;
		point.idx_[ 1 ] = GetIndex( corner[ 0 ], corner[ 1 ], corner[ 2 ] + 1 ) * 3;
		point.idx_[ 2 ] = GetIndex( corner[ 0 ], corner[ 1 ] + 1, corner[ 2 ] ) * 3;
		point.idx_[ 3 ] = GetIndex( corner[ 0 ], corner[ 1 ] + 1, corner[ 2 ] + 1 ) * 3;
		point.idx_[ 4 ] = GetIndex( corner[ 0 ] + 1, corner[ 1 ], corner[ 2 ] ) * 3;
		point.idx_[ 5 ] = GetIndex( corner[ 0 ] + 1, corner[ 1 ], corner[ 2 ] + 1 ) * 3;
		point.idx_[ 6 ] = GetIndex( corner[ 0 ] + 1, corner[ 1 ] + 1, corner[ 2 ] ) * 3;
		point.idx_[ 7 ] = GetIndex( corner[ 0 ] + 1, corner[ 1 ] + 1, corner[ 2 ] + 1 ) * 3;

		point.val_[ 0 ] = ( 1 - residual[ 0 ] ) * ( 1 - residual[ 1 ] ) * ( 1 - residual[ 2 ] );
		point.val_[ 1 ] = ( 1 - residual[ 0 ] ) * ( 1 - residual[ 1 ] ) * ( residual[ 2 ] );
		point.val_[ 2 ] = ( 1 - residual[ 0 ] ) * ( residual[ 1 ] ) * ( 1 - residual[ 2 ] );
		point.val_[ 3 ] = ( 1 - residual[ 0 ] ) * ( residual[ 1 ] ) * ( residual[ 2 ] );
		point.val_[ 4 ] = ( residual[ 0 ] ) * ( 1 - residual[ 1 ] ) * ( 1 - residual[ 2 ] );
		point.val_[ 5 ] = ( residual[ 0 ] ) * ( 1 - residual[ 1 ] ) * ( residual[ 2 ] );
		point.val_[ 6 ] = ( residual[ 0 ] ) * ( residual[ 1 ] ) * ( 1 - residual[ 2 ] );
		point.val_[ 7 ] = ( residual[ 0 ] ) * ( residual[ 1 ] ) * ( residual[ 2 ] );

		pt[ 3 ] /= unit_length_;
		pt[ 4 ] /= unit_length_;
		pt[ 5 ] /= unit_length_;
		point.nval_[ 0 ] = 
			- pt[ 3 ] * ( 1 - residual[ 1 ] ) * ( 1 - residual[ 2 ] ) 
			- pt[ 4 ] * ( 1 - residual[ 0 ] ) * ( 1 - residual[ 2 ] ) 
			- pt[ 5 ] * ( 1 - residual[ 0 ] ) * ( 1 - residual[ 1 ] );
		point.nval_[ 1 ] = 
			- pt[ 3 ] * ( 1 - residual[ 1 ] ) * ( residual[ 2 ] ) 
			- pt[ 4 ] * ( 1 - residual[ 0 ] ) * ( residual[ 2 ] ) 
			+ pt[ 5 ] * ( 1 - residual[ 0 ] ) * ( 1 - residual[ 1 ] );
		point.nval_[ 2 ] = 
			- pt[ 3 ] * ( residual[ 1 ] ) * ( 1 - residual[ 2 ] ) 
			+ pt[ 4 ] * ( 1 - residual[ 0 ] ) * ( 1 - residual[ 2 ] ) 
			- pt[ 5 ] * ( 1 - residual[ 0 ] ) * ( residual[ 1 ] );
		point.nval_[ 3 ] = 
			- pt[ 3 ] * ( residual[ 1 ] ) * ( residual[ 2 ] ) 
			+ pt[ 4 ] * ( 1 - residual[ 0 ] ) * ( residual[ 2 ] ) 
			+ pt[ 5 ] * ( 1 - residual[ 0 ] ) * ( residual[ 1 ] );
		point.nval_[ 4 ] = 
			  pt[ 3 ] * ( 1 - residual[ 1 ] ) * ( 1 - residual[ 2 ] ) 
			- pt[ 4 ] * ( residual[ 0 ] ) * ( 1 - residual[ 2 ] ) 
			- pt[ 5 ] * ( residual[ 0 ] ) * ( 1 - residual[ 1 ] );
		point.nval_[ 5 ] = 
			  pt[ 3 ] * ( 1 - residual[ 1 ] ) * ( residual[ 2 ] ) 
			- pt[ 4 ] * ( residual[ 0 ] ) * ( residual[ 2 ] ) 
			+ pt[ 5 ] * ( residual[ 0 ] ) * ( 1 - residual[ 1 ] );
		point.nval_[ 6 ] = 
			  pt[ 3 ] * ( residual[ 1 ] ) * ( 1 - residual[ 2 ] ) 
			+ pt[ 4 ] * ( residual[ 0 ] ) * ( 1 - residual[ 2 ] ) 
			- pt[ 5 ] * ( residual[ 0 ] ) * ( residual[ 1 ] );
		point.nval_[ 7 ] = 
			  pt[ 3 ] * ( residual[ 1 ] ) * ( residual[ 2 ] ) 
			+ pt[ 4 ] * ( residual[ 0 ] ) * ( residual[ 2 ] ) 
			+ pt[ 5 ] * ( residual[ 0 ] ) * ( residual[ 1 ] );

		point.n_[ 0 ] = pt[ 3 ];
		point.n_[ 1 ] = pt[ 4 ];
		point.n_[ 2 ] = pt[ 5 ];

		return true;
	}
};

