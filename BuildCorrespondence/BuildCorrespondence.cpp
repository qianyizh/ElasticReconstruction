// BuildCorrespondence.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "CorresApp.h"
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/common/time.h>

int print_help ()
{
	cout << "\nApplication parameters:" << endl;
	cout << "    --help, -h                      : print this message" << endl;
	cout << "    --traj <log_file>               : initialization, camera pose trajectory" << endl;
	cout << "    --num <num_of_fragments>        : use together with --traj" << endl;
	cout << "    --interval <interval>           : use together with --traj, default : 50" << endl;
	cout << "    --length <length>               : use together with --traj, default : 3.0" << endl;
	cout << "    --reg_traj <log_file>           : initialization, registration.log file, will overwrite --traj" << endl;
	cout << "    --registration                  : registration results are written into reg_output.log file" << endl;
	cout << "    --reg_dist <dist>               : distance threshold for registration, default 0.03" << endl;
	cout << "    --reg_ratio <ratio>             : correspondence points are at least <ratio> in each point cloud, default 0.25" << endl;
	cout << "    --reg_num <number>              : correspondence point number requirement, default 40,000" << endl;
	cout << "    --blasklist <blacklist_file>    : each line is the block we want to blacklist" << endl;
	cout << "    --save_xyzn                     : save point cloud into ascii file" << endl;
	cout << "    --output_information            : output the registration information matrix into reg_output.info" << endl;
	cout << "    --redux <log_file>              : use transformations in <log_file> as constraints" << endl;
	return 0;
}

int main(int argc, char * argv[])
{
	using namespace pcl::console;

	if ( argc == 1 ||  find_switch( argc, argv, "--help" ) || find_switch( argc, argv, "-h" ) ) {
		return print_help ();
	}

	CCorresApp app;

	if ( find_switch( argc, argv, "--save_xyzn" ) ) {
		app.ToggleSaveXYZN();
	}

	string log_file, reg_log_file, blacklist_file, redux_file;
	double reg_dist, reg_ratio;
	int reg_num, num;

	if ( ( parse_argument( argc, argv, "--traj", log_file ) > 0 && parse_argument( argc, argv, "--num", num ) > 0 ) 
		|| parse_argument( argc, argv, "--reg_traj", reg_log_file ) > 0 ) {
		pcl::ScopeTime time( "Registration All" );

		if ( parse_argument( argc, argv, "--reg_dist", reg_dist ) > 0 ) {
			app.reg_dist_ = reg_dist;
			app.dist_thresh_ = reg_dist / 2.0;
		}
		if ( parse_argument( argc, argv, "--reg_ratio", reg_ratio ) > 0 ) {
			app.reg_ratio_ = reg_ratio;
		}
		if ( parse_argument( argc, argv, "--reg_num", reg_num ) > 0 ) {
			app.reg_num_ = reg_num;
		}
		parse_argument( argc, argv, "--length", app.length_ );
		parse_argument( argc, argv, "--interval", app.interval_ );

		if ( reg_log_file.length() > 0 ) {
			app.LoadData( reg_log_file, -1 );
		} else {
			app.LoadData( log_file, num );
		}

		if ( parse_argument( argc, argv, "--blacklist", blacklist_file ) > 0 ) {
			app.Blacklist( blacklist_file );
		}
		if ( parse_argument( argc, argv, "--redux", redux_file ) > 0 ) {
			app.Redux( redux_file );
		}
		if ( find_switch( argc, argv, "--output_information" ) ) {
			app.output_information_ = true;
		}
		if ( find_switch( argc, argv, "--registration" ) ) {
			pcl::ScopeTime ttime( "Neat Registration" );
			app.Registration();
		}

		app.FindCorrespondence();

		app.Finalize();
	}

}

