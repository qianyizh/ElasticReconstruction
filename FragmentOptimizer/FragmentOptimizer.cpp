// FragmentOptimizer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "OptApp.h"
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/common/time.h>

int print_help ()
{
	cout << "\nApplication parameters:" << endl;
	cout << "    --help, -h                      : print this message" << endl;
	cout << "    --rgbdslam <log_file>           : rgbdslam.log/opt_output.log file, get ipose" << endl;
	cout << "    --registration <log_file>       : reg_output.log, invalid pair when frame_ == -1" << endl;
	cout << "    --dir <dir_prefix>              : dir prefix, place to loopup .xyzn files" << endl;
	cout << "    --num <number>                  : number of pieces, important parameter" << endl;
	cout << "    --weight <weight>               : 1.0 for nonrigid, 10000.0 for rigid" << endl;
	cout << "    --resolution <resolution>       : default - 8" << endl;
	cout << "    --length <length>               : default - 3.0" << endl;
	cout << "    --interval <interval>           : default - 50" << endl;
	cout << "    --iteration <max_number>        : default - 5" << endl;
	cout << "    --inner_iteration <max_number>  : default - 10" << endl;
	cout << "    --save_to <ctr_file>            : default - output.ctr" << endl;
	cout << "    --write_xyzn_sample <sample_num>: per <sample_num> write a point" << endl;
	cout << "    --blasklist <blacklist_file>    : each line is the block we want to blacklist" << endl;
	cout << "    --blacklistpair <threshold>     : threshold of accepting pairwise registration, default - 10000" << endl;
	cout << "    --switchable <weight>           : use switchable g2o, default weight 0.1" << endl;	
	cout << "    --ipose <log_file>              : get ipose from log file" << endl;
	cout << "    --verbose <verbpse_level>       : set verbosity level, from 0(none) to 5(verbose), default 3" << endl;
	cout << "Optimization options:" << endl;
	cout << "    --nonrigid                      : default, nonrigid alignment published in ICCV 2013" << endl;
	cout << "    --rigid                         : dense rigid optimization" << endl;
	cout << "    --slac                          : simultaneous localization and calibration, published in CVPR 2014" << endl;
	return 0;
}

int main(int argc, char * argv[])
{
	using namespace pcl::console;

	int verbosity_level = 3;
	parse_argument( argc, argv, "--verbose", verbosity_level );
	setVerbosityLevel( (VERBOSITY_LEVEL)verbosity_level );

	if ( argc == 1 || find_switch( argc, argv, "--help" ) || find_switch( argc, argv, "-h" ) ) {
		return print_help ();
	}

	COptApp app;
	std::string blacklist_file, ipose_file;

	if ( parse_argument( argc, argv, "--rgbdslam", app.rgbd_filename_ ) > 0 ) {
		app.rgbd_traj_.LoadFromFile( app.rgbd_filename_ );
	}
	if ( parse_argument( argc, argv, "--registration", app.reg_filename_ ) > 0 ) {
		app.reg_traj_.LoadFromFile( app.reg_filename_ );
	}
	parse_argument( argc, argv, "--dir", app.dir_prefix_ );
	parse_argument( argc, argv, "--init_ctr", app.init_ctr_file_ );

	parse_argument( argc, argv, "--num", app.num_ );
	parse_argument( argc, argv, "--weight", app.weight_ );
	parse_argument( argc, argv, "--resolution", app.resolution_ );
	parse_argument( argc, argv, "--length", app.length_ );
	parse_argument( argc, argv, "--interval", app.interval_ );
	parse_argument( argc, argv, "--iteration", app.max_iteration_ );
	parse_argument( argc, argv, "--inner_iteration", app.max_inner_iteration_ );
	parse_argument( argc, argv, "--save_to", app.ctr_filename_ );
	parse_argument( argc, argv, "--write_xyzn_sample", app.sample_num_ );
	parse_argument( argc, argv, "--blacklistpair", app.blacklist_pair_num_ );	

	if ( parse_argument( argc, argv, "--blacklist", blacklist_file ) > 0 ) {
		app.Blacklist( blacklist_file );
	}

	if ( parse_argument( argc, argv, "--ipose", ipose_file ) > 0 ) {
		app.IPoseFromFile( ipose_file );
	}

	{
		pcl::ScopeTime time( "Optimize All" );

		if ( find_switch( argc, argv, "--slac" ) ) {
			app.OptimizeSLAC();
		} else if ( find_switch( argc, argv, "--rigid") ) {
			app.OptimizeRigid();
		} else {
			app.OptimizeNonrigid();
		}
	}

	return 0;
}