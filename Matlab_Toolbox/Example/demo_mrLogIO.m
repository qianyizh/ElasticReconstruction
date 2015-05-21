addpath( '../Core' );

traj_gt = mrLoadLog( 'Data/traj_gt.log' );
traj_rigid = mrLoadLog( 'Data/traj_rigid.log' );

mrWriteLog( traj_gt, 'Data/temp.log' );
traj_gt = mrLoadLog( 'Data/temp.log' );
delete( 'Data/temp.log' );

figure(4328);
hold on;
mrDrawTraj( traj_gt, 'r-' );
mrDrawTraj( traj_rigid );
hold off;
