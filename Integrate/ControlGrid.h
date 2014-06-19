#pragma once
#include <vector>
#include <Eigen/Core>

struct Coordinate {
public:
	int idx_[ 8 ];
	float val_[ 8 ];
	float nval_[ 8 ];
};

class ControlGrid
{
public:
	ControlGrid(void);
	~ControlGrid(void);

public:
	std::vector< Eigen::Vector3f > ctr_;
	int resolution_;
	float length_;
	float unit_length_;
	int min_bound_[ 3 ];
	int max_bound_[ 3 ];

public:
	void Load( FILE * f, int res, float len );

public:
	void ResetBBox() { min_bound_[ 0 ] = min_bound_[ 1 ] = min_bound_[ 2 ] = 100000000; max_bound_[ 0 ] = max_bound_[ 1 ] = max_bound_[ 2 ] = -100000000; }
	void RegulateBBox( float vi, int i ) {
		int v0 = ( int )floor( vi / unit_length_ ) - 1;
		int v1 = ( int )ceil( vi / unit_length_ ) + 1;
		if ( v0 < min_bound_[ i ] ) {
			min_bound_[ i ] = v0;
		}
		if ( v1 > max_bound_[ i ] ) {
			max_bound_[ i ] = v1;
		}
	}
	inline int GetIndex( int i, int j, int k ) {
		return i + j * ( resolution_ + 1 ) + k * ( resolution_ + 1 ) * ( resolution_ + 1 );
	}
	inline bool GetCoordinate( const Eigen::Vector3f & pt, Coordinate & coo ) {
		int corner[ 3 ] = {
			( int )floor( pt( 0 ) / unit_length_ ),
			( int )floor( pt( 1 ) / unit_length_ ),
			( int )floor( pt( 2 ) / unit_length_ )
		};

		if ( corner[ 0 ] < 0 || corner[ 0 ] >= resolution_
			|| corner[ 1 ] < 0 || corner[ 1 ] >= resolution_
			|| corner[ 2 ] < 0 || corner[ 2 ] >= resolution_ )
			return false;

		float residual[ 3 ] = {
			pt( 0 ) / unit_length_ - corner[ 0 ],
			pt( 1 ) / unit_length_ - corner[ 1 ],
			pt( 2 ) / unit_length_ - corner[ 2 ]
		};
		// for speed, skip sanity check
		coo.idx_[ 0 ] = GetIndex( corner[ 0 ], corner[ 1 ], corner[ 2 ] );
		coo.idx_[ 1 ] = GetIndex( corner[ 0 ], corner[ 1 ], corner[ 2 ] + 1 );
		coo.idx_[ 2 ] = GetIndex( corner[ 0 ], corner[ 1 ] + 1, corner[ 2 ] );
		coo.idx_[ 3 ] = GetIndex( corner[ 0 ], corner[ 1 ] + 1, corner[ 2 ] + 1 );
		coo.idx_[ 4 ] = GetIndex( corner[ 0 ] + 1, corner[ 1 ], corner[ 2 ] );
		coo.idx_[ 5 ] = GetIndex( corner[ 0 ] + 1, corner[ 1 ], corner[ 2 ] + 1 );
		coo.idx_[ 6 ] = GetIndex( corner[ 0 ] + 1, corner[ 1 ] + 1, corner[ 2 ] );
		coo.idx_[ 7 ] = GetIndex( corner[ 0 ] + 1, corner[ 1 ] + 1, corner[ 2 ] + 1 );

		coo.val_[ 0 ] = ( 1 - residual[ 0 ] ) * ( 1 - residual[ 1 ] ) * ( 1 - residual[ 2 ] );
		coo.val_[ 1 ] = ( 1 - residual[ 0 ] ) * ( 1 - residual[ 1 ] ) * ( residual[ 2 ] );
		coo.val_[ 2 ] = ( 1 - residual[ 0 ] ) * ( residual[ 1 ] ) * ( 1 - residual[ 2 ] );
		coo.val_[ 3 ] = ( 1 - residual[ 0 ] ) * ( residual[ 1 ] ) * ( residual[ 2 ] );
		coo.val_[ 4 ] = ( residual[ 0 ] ) * ( 1 - residual[ 1 ] ) * ( 1 - residual[ 2 ] );
		coo.val_[ 5 ] = ( residual[ 0 ] ) * ( 1 - residual[ 1 ] ) * ( residual[ 2 ] );
		coo.val_[ 6 ] = ( residual[ 0 ] ) * ( residual[ 1 ] ) * ( 1 - residual[ 2 ] );
		coo.val_[ 7 ] = ( residual[ 0 ] ) * ( residual[ 1 ] ) * ( residual[ 2 ] );

		return true;
	}
	inline void GetPosition( const Coordinate & coo, Eigen::Vector3f & pos ) {
		pos = coo.val_[ 0 ] * ctr_[ coo.idx_[ 0 ] ] + coo.val_[ 1 ] * ctr_[ coo.idx_[ 1 ] ]
			+ coo.val_[ 2 ] * ctr_[ coo.idx_[ 2 ] ] + coo.val_[ 3 ] * ctr_[ coo.idx_[ 3 ] ]
			+ coo.val_[ 4 ] * ctr_[ coo.idx_[ 4 ] ] + coo.val_[ 5 ] * ctr_[ coo.idx_[ 5 ] ]
			+ coo.val_[ 6 ] * ctr_[ coo.idx_[ 6 ] ] + coo.val_[ 7 ] * ctr_[ coo.idx_[ 7 ] ];
	}
};

