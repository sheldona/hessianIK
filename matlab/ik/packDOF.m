function [ x ] = packDOF( skel, mot, frame )
%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%PACK Pack motion frame into dof vector.
%
ndof = computeDOF(skel);
x = zeros(ndof, 1);
idx = 1;
for i = 1:skel.njoints
    if( strcmp(skel.nodes(i).boneName,'lthumb') || ...
        strcmp(skel.nodes(i).boneName,'rthumb') )
        % do nothing
    elseif( ~isempty(mot.rotationEuler{i}) )
        sz = size(skel.nodes(i).DOF,1);
        if( i == 1 )
            x(1:3) = mot.rootTranslation(:,frame);
            x(4:6) = mot.rotationEuler{1}(:,frame);
        else 
            x(idx:idx+sz-1,:) = mot.rotationEuler{i}(:,frame);
        end
        idx = idx + sz;
    end
end

end

