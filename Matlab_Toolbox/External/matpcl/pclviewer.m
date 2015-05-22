%PCLVIEWER View a point cloud using PCL
%
% PCLVIEWER(P) writes the point cloud P (MxN) to a temporary file and invokes
% the PCL point cloud viewer for fast display and visualization.  The columns of P
% represent the 3D points.
%
% If M=3 then the rows are x, y, z.
% If M=6 then the rows are x, y, z, R, G, B where R,G,B are in the range 0
% to 1.
%
% PCLVIEWER(P, ARGS) as above but the optional arguments ARGS are passed to the
% PCL viewer.  For example:
%
%         pclviewer( rand(3,1000), '-ps 2 -ax 1' )
%
% Notes::
% - Only the "x y z" and "x y z rgb" field formats are currently supported.
% - The file is written in ascii format.
% - When viewing colored point clouds in pcl_viewer remember to toggle to 
%
% See also savepcd, lspcd, readpcd.
%
% Copyright (C) 2013, by Peter I. Corke

% TODO
% - add color


function pclviewer(points, args)
    
    % change the next line to suit your operating system
    viewer = '/usr/local/bin/pcl_viewer.app/Contents/MacOS/pcl_viewer';

     
    pointfile = [tempname '.pcd'];
    
    if nargin < 2
        args = '';
    end
    
    savepcd(pointfile, points);
    
    system(sprintf('head -20 %s', pointfile));
    
    system(sprintf('%s %s %s &', ...
        viewer, pointfile, args));
   
    pause(1)
    delete(pointfile);
    