#pragma once

#include "external/Eigen/Core"
#include "external/Eigen/SparseCore"
#include "external/Eigen/Dense"
#include "external/Eigen/IterativeLinearSolvers"
#include "external/Eigen/CholmodSupport"
#include "external/unsupported/Eigen/SparseExtra"
#include <hash_map>
#include <vector>

typedef Eigen::Triplet< double > Triplet;
typedef std::vector< Triplet > TripletVector;
typedef stdext::hash_map< int, int > HashMap;
typedef stdext::hash_map< int, int >::const_iterator HashMapIterator;
typedef pair< int, int > IntPair;

class HashSparseMatrix
{
public:
	HashSparseMatrix( int ioffset, int joffset );
	~HashSparseMatrix(void);

public:
	HashMap map_;
	int ioffset_, joffset_;

public:
	void AddHessian( int idx[], double val[], int n, TripletVector & data );
	void AddHessian( int idx1[], double val1[], int n1, int idx2[], double val2[], int n2, TripletVector & data );
	void AddHessian2( int idx[], double val[], TripletVector & data );
	void Add( int i, int j, double value, TripletVector & data );

	void AddJb( int idx[], double val[], int n, double b, Eigen::VectorXd & Jb );
	void AddJb( int i, double value, double b, Eigen::VectorXd & Jb );
};

