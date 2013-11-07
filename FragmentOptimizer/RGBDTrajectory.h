#include <vector>
#include <fstream>
#include "external/Eigen/Core"

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
