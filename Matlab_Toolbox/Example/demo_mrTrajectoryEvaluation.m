addpath( '../Core' );

traj_gt = mrLoadLog( 'Data/Trajectory/traj_gt.log' );
traj_rigid = mrLoadLog( 'Data/Trajectory/traj_rigid.log' );

[ rmse, trans ] = mrEvaluateTrajectory( traj_rigid, traj_gt );

figure(4328);
clf;
hold on;
mrDrawTrajectory( traj_gt, 'k-', trans );
mrDrawTrajectory( traj_rigid, 'r-', eye( 4 ) );
%mrDrawTrajectory( traj_gt, 'k-', eye( 4 ) );
%mrDrawTrajectory( traj_rigid, 'r-', trans ^ -1 );
hold off;
legend( 'Ground truth trajectory', 'Estimated trajectory', ...
    'Location', 'northwest' );
