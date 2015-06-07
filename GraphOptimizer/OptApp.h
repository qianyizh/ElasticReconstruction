#pragma once

#include <g2o/types/slam3d/se3quat.h>
#include "helper.h"

class COptApp
{
public:
	COptApp(void);
	~COptApp(void);

public:
	RGBDTrajectory odometry_traj_;
	RGBDInformation odometry_info_;
	RGBDTrajectory loop_traj_;
	RGBDInformation loop_info_;
	
	RGBDTrajectory pose_traj_;
	RGBDTrajectory loop_remain_traj_;
	RGBDTrajectory refine_traj_;

	// input
	std::string odometry_log_file_;
	std::string loop_log_file_;
	std::string odometry_info_file_;
	std::string loop_info_file_;
	std::string method_;
	double weight_;
	int max_iteration_;
	
	// output
	std::string pose_log_file_;
	std::string loop_remain_log_file_;
	std::string refine_log_file_;

public:
	bool Init();
	void OptimizeSwitchable();
	void OptimizeEM();

private:
	Eigen::Matrix4d G2O2Matrix4d( const g2o::SE3Quat se3 ) {
		Eigen::Matrix4d m = se3.to_homogeneous_matrix(); //_Matrix< 4, 4, double >
		return m;
	}

	g2o::SE3Quat Eigen2G2O( const Eigen::Matrix4d & eigen_mat ) {
		Eigen::Affine3d eigen_transform( eigen_mat );
		Eigen::Quaterniond eigen_quat( eigen_transform.rotation() );
		Eigen::Vector3d translation( eigen_mat( 0, 3 ), eigen_mat( 1, 3 ), eigen_mat( 2, 3 ) );
		g2o::SE3Quat result( eigen_quat, translation );
		return result;
	}
};

