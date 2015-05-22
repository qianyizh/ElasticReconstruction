addpath( '../Core' );

traj_gt = mrLoadLog( 'Data/Trajectory/traj_gt.log' );
traj_rigid = mrLoadLog( 'Data/Trajectory/traj_rigid.log' );

mrWriteLog( traj_gt, 'Data/temp.log' );
traj_gt = mrLoadLog( 'Data/temp.log' );
delete( 'Data/temp.log' );

figure(4328);
clf;
hold on;
mrDrawTraj( traj_gt, 'k-' );
mrDrawTraj( traj_rigid, 'r-' );
hold off;
legend( 'Ground truth trajectory', 'Estimated trajectory', ...
    'Location', 'northwest' );

reg_log_gt = mrLoadLog( 'Data/RegistrationEvaluation/registration_gt.log' );
reg_info_gt = mrLoadInfo( 'Data/RegistrationEvaluation/registration_gt.info' );
mrWriteInfo( reg_info_gt, 'Data/temp.info' );
reg_info_gt = mrLoadInfo( 'Data/temp.info' );
delete( 'Data/temp.info' );
