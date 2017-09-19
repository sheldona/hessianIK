function [mot] = unpackDOF( x, skel, mot, frame )
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%UNPACK Unpack dof vector into motion frame
%
idx = 1;
for i = 1:skel.njoints
    sz = size(skel.nodes(i).DOF,1);
    if( strcmp(skel.nodes(i).boneName,'lthumb') || ...
        strcmp(skel.nodes(i).boneName,'rthumb') )
        mot.rotationEuler{i,1}(:,frame) = zeros(sz,1);
    else
        if( sz > 0 )
            if( i == 1 )   % root
                mot.rootTranslation(:,frame) = x(1:3);
                mot.rotationEuler{1,1}(:,frame) = x(4:6);
            else
                mot.rotationEuler{i,1}(:,frame) = x(idx:idx+sz-1);
            end
            idx = idx + sz;
        end
    end
end

end

