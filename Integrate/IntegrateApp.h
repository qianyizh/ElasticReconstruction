#pragma once
#include <boost/filesystem.hpp>
#include <pcl/io/grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/common/time.h>

#include "TSDFVolume.h"

struct SampledScopeTime : public pcl::StopWatch
{          
	enum { EACH = 33 };
	SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
	~SampledScopeTime() { 
		static int i_ = 0; 
		time_ms_ += pcl::StopWatch::getTime ();
		if ( i_ % EACH == 0 && i_ ) {
			cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )" << endl;
			time_ms_ = 0;        
		}
		++i_;
	}
private:    
	int & time_ms_;    
};

class CIntegrateApp
{
private:
	std::vector< unsigned short > depth_;
	std::vector< float > scaled_depth_;
	int cols_, rows_;

	pcl::Grabber & capture_;
	bool use_device_;
	bool exit_;
	bool registration_;
	int frame_id_;
	int time_ms_;

	boost::mutex data_ready_mutex_;
	boost::condition_variable data_ready_cond_;

	RGBDTrajectory traj_;

	TSDFVolume volume_;
	int frame_id_;

public:
	CIntegrateApp( pcl::Grabber & source, bool use_device );
	~CIntegrateApp(void);

public:
	void StartMainLoop( bool triggered_capture );

private:
	void Execute( bool has_data );

private:		// callback functions
	void source_cb2( const boost::shared_ptr< openni_wrapper::Image >& image_wrapper, const boost::shared_ptr< openni_wrapper::DepthImage >& depth_wrapper, float );
	void source_cb2_trigger( const boost::shared_ptr< openni_wrapper::Image >& image_wrapper, const boost::shared_ptr< openni_wrapper::DepthImage >& depth_wrapper, float );
	void print_help();
};

