// GraphOptimizer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "OptApp.h"
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;

int main(int argc, char * argv[])
{
	COptApp app;

	po::options_description generic_option( "Generic Options" );
	generic_option.add_options()
		( "help,h", "print this message" )
		( "function,f", po::value< string >( &app.method_ )->default_value( "switchable" ), "possible choices: switchable/em" )
		( "weight,w", po::value< double >( &app.weight_ )->default_value( 1.0 ), "weight for switchable constraint penalty" )
		( "iteration,i", po::value< int >( &app.max_iteration_ )->default_value( 100 ), "maximum optimization iteration" )
	;
		std::string odometry_log_file_;
	std::string loop_log_file_;
	std::string odometry_info_file_;
	std::string loop_info_file_;
	std::string method_;
	double weight_;
	int max_iteration_;
	
	// output
	po::options_description io_option( "Input/Output Options (configured automatically if not otherwise specified)" );
	io_option.add_options()
		( "odometry", po::value< string >( &app.odometry_log_file_ )->default_value( "odometry.log" ), "odometry transformations" )
		( "odometryinfo", po::value< string >( &app.odometry_info_file_ )->default_value( "odometry.info" ), "odometry information matrices, optional" )
		( "loop", po::value< string >( &app.loop_log_file_ )->default_value( "loop.log" ), "loop closure transformations" )
		( "loopinfo", po::value< string >( &app.loop_info_file_ )->default_value( "loop.info" ), "loop closure information matrices, optional" )
		( "pose", po::value< string >( &app.pose_log_file_ )->default_value( "opt_output.log" ), "output poses of fragments" )
		( "keep", po::value< string >( &app.loop_remain_log_file_ )->default_value( "loop_remain.log" ), "output pruned loop closure transformations" )
		( "refine", po::value< string >( &app.refine_log_file_ )->default_value( "refine.log" ), "loop closure and odometry edges that need to be refined" )
		;

	po::options_description arg_option;
	arg_option.add( generic_option ).add( io_option );

	po::variables_map vm;
	po::store( po::parse_command_line( argc, argv, arg_option ), vm );
	po::notify( vm );

	if ( argc == 1 || vm.count( "help" ) ) {
		cout << arg_option << endl;
		return 1;
	}

	if ( app.method_.compare( "switchable" ) == 0 ) {
		if ( app.Init() ) {
			app.OptimizeSwitchable();
		}
	} else if ( app.method_.compare( "em" ) == 0 ) {
		if ( app.Init() ) {
			app.OptimizeEM();
		}
	}

	return 0;
}

