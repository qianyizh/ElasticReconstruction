#pragma once

#include <vector>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>

struct FramedTransformation {
	int id1_;
	int id2_;
	int frame_;
	Eigen::Matrix4d transformation_;
	FramedTransformation( int id1, int id2, int f, Eigen::Matrix4d t )
		: id1_( id1 ), id2_( id2 ), frame_( f ), transformation_( t ) 
	{}
};

struct RGBDTrajectory {
	std::vector< FramedTransformation > data_;
	int index_;

	void LoadFromFile( std::string filename ) {
		data_.clear();
		index_ = 0;
		int id1, id2, frame;
		Eigen::Matrix4d trans;
		FILE * f = fopen( filename.c_str(), "r" );
		if ( f != NULL ) {
			char buffer[1024];
			while ( fgets( buffer, 1024, f ) != NULL ) {
				if ( strlen( buffer ) > 0 && buffer[ 0 ] != '#' ) {
					sscanf( buffer, "%d %d %d", &id1, &id2, &frame);
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(0,0), &trans(0,1), &trans(0,2), &trans(0,3) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(1,0), &trans(1,1), &trans(1,2), &trans(1,3) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(2,0), &trans(2,1), &trans(2,2), &trans(2,3) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(3,0), &trans(3,1), &trans(3,2), &trans(3,3) );
					data_.push_back( FramedTransformation( id1, id2, frame, trans ) );
				}
			}
			fclose( f );
		}
	}
	void SaveToFile( std::string filename ) {
		FILE * f = fopen( filename.c_str(), "w" );
		for ( int i = 0; i < ( int )data_.size(); i++ ) {
			Eigen::Matrix4d & trans = data_[ i ].transformation_;
			fprintf( f, "%d\t%d\t%d\n", data_[ i ].id1_, data_[ i ].id2_, data_[ i ].frame_ );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(0,0), trans(0,1), trans(0,2), trans(0,3) );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(1,0), trans(1,1), trans(1,2), trans(1,3) );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(2,0), trans(2,1), trans(2,2), trans(2,3) );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(3,0), trans(3,1), trans(3,2), trans(3,3) );
		}
		fclose( f );
	}
};

typedef Eigen::Matrix< double, 6, 6, Eigen::RowMajor > InformationMatrix;

struct FramedInformation {
	int id1_;
	int id2_;
	int frame_;
	InformationMatrix information_;
	FramedInformation( int id1, int id2, int f, InformationMatrix t )
		: id1_( id1 ), id2_( id2 ), frame_( f ), information_( t ) 
	{}
};

struct RGBDInformation {
	std::vector< FramedInformation > data_;

	void LoadFromFile( std::string filename ) {
		data_.clear();
		int id1, id2, frame;
		InformationMatrix info;
		FILE * f = fopen( filename.c_str(), "r" );
		if ( f != NULL ) {
			char buffer[1024];
			while ( fgets( buffer, 1024, f ) != NULL ) {
				if ( strlen( buffer ) > 0 && buffer[ 0 ] != '#' ) {
					sscanf( buffer, "%d %d %d", &id1, &id2, &frame);
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf %lf %lf", &info(0,0), &info(0,1), &info(0,2), &info(0,3), &info(0,4), &info(0,5) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf %lf %lf", &info(1,0), &info(1,1), &info(1,2), &info(1,3), &info(1,4), &info(1,5) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf %lf %lf", &info(2,0), &info(2,1), &info(2,2), &info(2,3), &info(2,4), &info(2,5) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf %lf %lf", &info(3,0), &info(3,1), &info(3,2), &info(3,3), &info(3,4), &info(3,5) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf %lf %lf", &info(4,0), &info(4,1), &info(4,2), &info(4,3), &info(4,4), &info(4,5) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf %lf %lf", &info(5,0), &info(5,1), &info(5,2), &info(5,3), &info(5,4), &info(5,5) );
					data_.push_back( FramedInformation( id1, id2, frame, info ) );
				}
			}
			fclose( f );
		}
	}
	void SaveToFile( std::string filename ) {
		FILE * f = fopen( filename.c_str(), "w" );
		for ( int i = 0; i < ( int )data_.size(); i++ ) {
			InformationMatrix & info = data_[ i ].information_;
			fprintf( f, "%d\t%d\t%d\n", data_[ i ].id1_, data_[ i ].id2_, data_[ i ].frame_ );
			fprintf( f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0,0), info(0,1), info(0,2), info(0,3), info(0,4), info(0,5) );
			fprintf( f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1,0), info(1,1), info(1,2), info(1,3), info(1,4), info(1,5) );
			fprintf( f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2,0), info(2,1), info(2,2), info(2,3), info(2,4), info(2,5) );
			fprintf( f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3,0), info(3,1), info(3,2), info(3,3), info(3,4), info(3,5) );
			fprintf( f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4,0), info(4,1), info(4,2), info(4,3), info(4,4), info(4,5) );
			fprintf( f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5,0), info(5,1), info(5,2), info(5,3), info(5,4), info(5,5) );
		}
		fclose( f );
	}
};
