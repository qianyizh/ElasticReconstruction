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
	cout << "    --traj <log_file>               : initialization, registration.log file" << endl;
	cout << "    --registration                  : registration results are written into reg_output.log file" << endl;
	cout << "    --reg_dist <dist>               : distance threshold for registration, default 0.05" << endl;
	cout << "    --reg_ratio <ratio>             : correspondence points are at least <ratio> in each point cloud, default 0.3" << endl;
	cout << "    --reg_num <number>              : correspondence point number requirement, default 50,000" << endl;
	cout << "    --blasklist <blacklist_file>    : each line is the block we want to blacklist" << endl;
	cout << "    --save_xyzn                     : save point cloud into ascii file" << endl;
	cout << "    --redux <log_file>              : use transformations in <log_file> as constraints" << endl;
	return 0;
}

int main(int argc, char * argv[])
{
	using namespace pcl::console;

	if ( find_switch( argc, argv, "--help" ) || find_switch( argc, argv, "-h" ) ) {
		return print_help ();
	}

	CCorresApp app;

	if ( find_switch( argc, argv, "--save_xyzn" ) ) {
		app.ToggleSaveXYZN();
	}

	string log_file, blacklist_file, redux_file;
	double reg_dist, reg_ratio;
	int reg_num;

	if ( parse_argument( argc, argv, "--traj", log_file ) > 0 ) {
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

		app.LoadData( log_file );

		if ( parse_argument( argc, argv, "--blacklist", blacklist_file ) > 0 ) {
			app.Blacklist( blacklist_file );
		}
		if ( parse_argument( argc, argv, "--redux", redux_file ) > 0 ) {
			app.Redux( redux_file );
		}
		if ( find_switch( argc, argv, "--registration" ) ) {
			pcl::ScopeTime ttime( "Neat Registration" );
			app.Registration();
		}

		app.FindCorrespondence();

		app.Finalize();
	}

}

