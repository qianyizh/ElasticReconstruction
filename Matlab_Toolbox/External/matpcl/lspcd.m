%LSPCD List attributes of PCD format files
%
% LSPCD() list the attributes of all .PCD files in the current folder.
%
% LSPCD(FILESPEC) as above but list only files that match FILESPEC which 
% might contain a directory name and/or a wildcard.
%
%
% See also pclviewer, loadpcd.
%
% Copyright (C) 2013, by Peter I. Corke


function lspcd(name)
    
    % default to .pcd files in current dir
    if nargin < 1
        name = '*.pcd';
    end
    
    
    path = fileparts(name);  % get the common path
    files = dir(name);  % get all the matching files
    
    for file=files'
        header(path, file.name);
    end
    
end

function header(dir, file)
    
    % build the full path to the file
    fname = fullfile(dir, file);
    
    fp = fopen(fname, 'r');
    version = [];

    while true
        line = fgetl(fp);
        
        if line(1) == '#'
            continue;
        end
        
        [field,remain] = strtok(line, ' \t');
        remain = strtrim(remain);
        
        switch field
            case 'VERSION'
                version = remain;
            case 'FIELDS'
                fields = remain;
            case 'TYPE'
                type = remain;
            case 'WIDTH'
                width = str2num(remain);
            case 'HEIGHT'
                height = str2num(remain);
            case 'POINTS'
                npoints = str2num(remain);
            case 'SIZE'
                siz = str2num(remain);
            case 'COUNT'
                count = str2num(remain);
            case 'DATA'
                mode = remain;
                break;
            otherwise
                fprintf('unknown field %s\n', field);
        end
    end
    
    % if no version field we'll assume it's not a PCD file
    if isempty(version)
        return;
    end
    
    if height == 1
        org = 'unorganized';
    else
        org = 'organized';
    end
    fprintf('%s: %s, %s, <%s> %dx%d\n', ...
        fname, mode, org, fields, width, height);
    fprintf('  %s; %s\n', type, num2str(siz));
    
    fclose(fp);
end