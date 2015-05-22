addpath( '../Core' );

traj_gt = mrLoadLog( 'Data/Trajectory/traj_gt.log' );
traj_rigid = mrLoadLog( 'Data/Trajectory/traj_rigid.log' );

[ rmse, trans ] = mrEvaluateTraj( traj_rigid, traj_gt );

figure(4328);
clf;
hold on;
mrDrawTraj( traj_gt, 'k-', trans );
mrDrawTraj( traj_rigid, 'r-', eye( 4 ) );
%mrDrawTraj( traj_gt, 'k-', eye( 4 ) );
%mrDrawTraj( traj_rigid, 'r-', trans ^ -1 );
hold off;
legend( 'Ground truth trajectory', 'Estimated trajectory', ...
    'Location', 'northwest' );
