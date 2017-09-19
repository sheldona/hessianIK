function [ ndof, dofMap ] = computeDOF( skel )
%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%COMPUTEDOF Returns the number of degrees-of-freedom in skel. This function
% also returns the mapping from bone id to index in the dof array.
%
% INPUT:  
%   skel - The skeleton data structure following HDM05 sepecification
% OUTPUT: 
%   ndof - The dof count.
%   dofMap - A two column vector containing the dof offset (first column)
%      and size (second column) for each joint in the skeleton. 

ndof = 0;
if( nargout > 1 )
    dofMap = zeros(skel.njoints,2);
end
for i = 1:skel.njoints
    if( ~strcmp(skel.nodes(i).boneName,'lthumb') && ...
        ~strcmp(skel.nodes(i).boneName,'rthumb') )
        sz = size(skel.nodes(i).DOF,1);
        if( nargout > 1 )
            dofMap(i,:) = [ ndof+1 sz];
        end
        ndof = ndof + sz;
    end
end

end

