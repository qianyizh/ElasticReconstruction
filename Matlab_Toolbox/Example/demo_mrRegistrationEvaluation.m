addpath( '../Core' );

names = { 'opencv', '4pcs', 'super4pcs', 'pcl', 'pcl_modified' };
basedir = 'Data/RegistrationEvaluation/';
dirnames = { 'livingroom1', 'livingroom2', 'office1', 'office2' };

names_num = size( names, 2 );
synth_num = 4;

recall = zeros( synth_num, names_num );
precision = zeros( synth_num, names_num );

for i = 1 : synth_num
    gt = mrLoadLog( [ basedir, dirnames{i}, '/gt.log' ] );
    gt_info = mrLoadInfo( [ basedir, dirnames{i}, '/gt.info' ] );
    for k = 1 : names_num
        result = mrLoadLog( [ basedir, dirnames{i}, '/', names{k}, '.log' ] );
        [ recall( i, k ), precision( i, k ) ] = ...
            mrEvaluateRegistration( result, gt, gt_info );
    end
end

eval_res = [ mean( recall ); mean( precision ) ];

fprintf( '%15s :\tRecall\tPrecision\n', 'Method' );
fprintf( '----------------------------------------\n' );
for k = 1 : names_num
    fprintf( '%15s :\t%3.1f%%\t%3.1f%%\n', ...
        names{k}, eval_res( 1, k ) * 100, eval_res( 2, k ) * 100 );
end