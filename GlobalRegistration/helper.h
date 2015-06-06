#pragma once

#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <pcl/console/print.h>

using namespace std;

struct Configuration {
public:
	bool vis_;
	bool aux_data_;
	bool estimate_normal_;
	bool smart_swap_;
	int max_iteration_;
	int num_of_samples_;
	int correspondence_randomness_;
	int pcl_verbose_;
	float edge_similarity_;
	float resample_leaf_;
	float max_correspondence_distance_;
	float inlier_fraction_;
	int inlier_number_;
	float angle_difference_;
	float normal_radius_;
	float feature_radius_;

public:
	Configuration()
		: vis_( false )
		, aux_data_( false )
		, estimate_normal_( true )
		, smart_swap_( true )
		, pcl_verbose_( 3 )
		, max_iteration_( 4000000 )
		, num_of_samples_( 4 )
		, correspondence_randomness_( 2 )
		, edge_similarity_( 0.9f )
		, resample_leaf_( 0.05f )
		, max_correspondence_distance_( 0.075f )
		, inlier_fraction_( 0.33f )
		, inlier_number_( 30000 )
		, angle_difference_( 0.52359878 )
		, normal_radius_( 0.1 )
		, feature_radius_( 0.25 )
	{
		if ( boost::filesystem::exists( "alignment.config" ) ) {
			FILE * f = fopen( "alignment.config", "r" );
			if ( f != NULL ) {
				char buffer[ 1024 ];
				while ( fgets( buffer, 1024, f ) != NULL ) {
					std::string str( buffer );
					std::stringstream ss( str );
					std::string item1, item2;
					std::getline( ss, item1, '=' );
					std::getline( ss, item2 );

					if ( item1.compare( "visualization" ) == 0 ) {
						vis_ = ( item2.compare( "true" ) == 0 );
						cout << "visualization = " << vis_ << endl;
					}
					if ( item1.compare( "estimate_normal" ) == 0 ) {
						estimate_normal_ = ( item2.compare( "true" ) == 0 );
						cout << "estimate_normal = " << estimate_normal_ << endl;
					}
					if ( item1.compare( "aux_data" ) == 0 ) {
						aux_data_ = ( item2.compare( "true" ) == 0 );
						cout << "aux_data = " << aux_data_ << endl;
					}
					if ( item1.compare( "smart_swap" ) == 0 ) {
						smart_swap_ = ( item2.compare( "true" ) == 0 );
						cout << "smart_swap = " << smart_swap_ << endl;
					}
					if ( item1.compare( "max_iteration" ) == 0 ) {
						max_iteration_ = boost::lexical_cast< int >( item2 );
						cout << "max_iteration = " << max_iteration_ << endl;
					}
					if ( item1.compare( "num_of_samples" ) == 0 ) {
						num_of_samples_ = boost::lexical_cast< int >( item2 );
						cout << "num_of_samples = " << num_of_samples_ << endl;
					}
					if ( item1.compare( "correspondence_randomness" ) == 0 ) {
						correspondence_randomness_ = boost::lexical_cast< int >( item2 );
						cout << "correspondence_randomness = " << correspondence_randomness_ << endl;
					}
					if ( item1.compare( "edge_similarity" ) == 0 ) {
						edge_similarity_ = boost::lexical_cast< float >( item2 );
						cout << "edge_similarity = " << edge_similarity_ << endl;
					}
					if ( item1.compare( "resample_leaf" ) == 0 ) {
						resample_leaf_ = boost::lexical_cast< float >( item2 );
						cout << "resample_leaf = " << resample_leaf_ << endl;
					}
					if ( item1.compare( "max_correspondence_distance" ) == 0 ) {
						max_correspondence_distance_ = boost::lexical_cast< float >( item2 );
						cout << "max_correspondence_distance = " << max_correspondence_distance_ << endl;
					}
					if ( item1.compare( "inlier_fraction" ) == 0 ) {
						inlier_fraction_ = boost::lexical_cast< float >( item2 );
						cout << "inlier_fraction = " << inlier_fraction_ << endl;
					}
					if ( item1.compare( "inlier_number" ) == 0 ) {
						inlier_number_ = boost::lexical_cast< int >( item2 );
						cout << "inlier_number = " << inlier_number_ << endl;
					}
					if ( item1.compare( "angle_difference" ) == 0 ) {
						angle_difference_ = boost::lexical_cast< float >( item2 );
						cout << "angle_difference = " << angle_difference_ << endl;
					}
					if ( item1.compare( "normal_radius" ) == 0 ) {
						normal_radius_ = boost::lexical_cast< float >( item2 );
						cout << "normal_radius = " << normal_radius_ << endl;
					}
					if ( item1.compare( "feature_radius" ) == 0 ) {
						feature_radius_ = boost::lexical_cast< float >( item2 );
						cout << "feature_radius = " << feature_radius_ << endl;
					}
					if ( item1.compare( "pcl_verbose" ) == 0 ) {
						pcl_verbose_ = boost::lexical_cast< int >( item2 );
						cout << "pcl_verbose = " << pcl_verbose_ << endl;
						pcl::console::setVerbosityLevel( pcl::console::VERBOSITY_LEVEL( pcl_verbose_ ) );
					}
				}
			}
		} else {
			cout << "alignment.config not found! Use default parameters." << endl;
		}
	}
};

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
