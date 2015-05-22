%LZFD  LZF decompression
%
% OUT = LZFD(IN) is the decompressed version of the uint8 array IN.
%
% OUT = LZFD(IN, LEN) as above but sets the internal working buffer to length
% LEN which should exceed the expected uncompressed data size.
%
% Notes::
% - LZF is an algorithm that is efficient and gives reasonable compression ratios
% - If LEN is not specified 2*length(IN) is used.  Better to overestimate to save
%   MATLAB continually extending the array.
%
% Reference::
% - C source code lizf_d.c from liblzf available from http://software.schmorp.de/pkg/liblzf
%
% Author::
% - Peter Corke

% Copyright (C) 2013 Peter Corke

function out = lzfd(in, outlen)
    
    if nargin < 2
        outlen = 2 * length(in);
    end
    
    ip = 1;  % input pointer, range 1 to length(in)
    op = 1;  % output pointer, range 1 to outlen
    
    % preallocate decompressed data storage
    out = zeros(1, outlen, 'uint8');
    
    while (1)
        
        ctrl = cast(in(ip), 'uint32'); ip = ip + 1;
        
        if ctrl < 32
            % literal run
            ctrl = ctrl+1;

            % lzf_movsb(op, ip, ctrl)
            out(op:op+ctrl-1) = in(ip:ip+ctrl-1); 
            ip = ip + ctrl;
            op = op + ctrl;
            
        else
            % back reference
            len = bitshift(ctrl, -5);
            ref = op - bitshift(bitand(ctrl, 31), 8) - 1;
            
            if len == 7
                len = len + cast(in(ip), 'uint32'); ip = ip + 1;
            end
            
            ref = ref - cast(in(ip), 'uint32'); ip = ip + 1;
                     
            % lzf_movsb(op, ref, len)
            len = len + 2;

            try
                out(op:op+len-1) = out(ref:ref+len-1);
            catch
                % extend the buffer used for decompression
                out = [out zeros(1, length(in), 'uint8')];
                outlen = length(out);
                out(op:op+len-1) = out(ref:ref+len-1);
            end
            op = op + len;
        end
        
        if ip >= length(in)  % are we done yet?
            break
        end
    end
    
    out = out(1:op-1);  % return the valid data
    
end