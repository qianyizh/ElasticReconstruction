#include "StdAfx.h"
#include "HashSparseMatrix.h"


HashSparseMatrix::HashSparseMatrix( int ioffset, int joffset )
{
	ioffset_ = ioffset;
	joffset_ = joffset;
}


HashSparseMatrix::~HashSparseMatrix(void)
{
}

void HashSparseMatrix::AddJb( int i, double value, double b, Eigen::VectorXd & Jb )
{
	Jb( i ) += value * b;
}

void HashSparseMatrix::AddJb( int idx[], double val[], int n, double b, Eigen::VectorXd & Jb )
{
	for ( int i = 0; i < n; i++ ) {
		AddJb( idx[ i ], val[ i ], b, Jb );
	}
}

void HashSparseMatrix::Add( int i, int j, double value, TripletVector & data )
{
	int key = i + j * 65536;
	HashMapIterator it = map_.find( key );
	if ( it == map_.end() ) {
		map_.insert( IntPair( key, data.size() ) );
		data.push_back( Triplet( i + ioffset_, j + joffset_, value ) );
	} else {
		data[ it->second ].m_value += value;
	}
}

void HashSparseMatrix::AddHessian( int idx[], double val[], int n, TripletVector & data )
{
	for ( int i = 0; i < n; i++ ) {
		for ( int j = 0; j < n; j++ ) {
			Add( idx[ i ], idx[ j ], val[ i ] * val[ j ], data );
		}
	}
}

void HashSparseMatrix::AddHessian2( int idx[], double val[], TripletVector & data )
{
	AddHessian( idx, val, 2, data );
	idx[ 0 ] ++; idx[ 1 ]++;
	AddHessian( idx, val, 2, data );
	idx[ 0 ] ++; idx[ 1 ]++;
	AddHessian( idx, val, 2, data );
	idx[ 0 ] -= 2; idx[ 1 ] -= 2;
}

void HashSparseMatrix::AddHessian( int idx1[], double val1[], int n1, int idx2[], double val2[], int n2, TripletVector & data )
{
	for ( int i = 0; i < n1; i++ ) {
		for ( int j = 0; j < n2; j++ ) {
			Add( idx1[ i ], idx2[ j ], val1[ i ] * val2[ j ], data );
		}
	}
}
