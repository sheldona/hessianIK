function [lb,ub] = computeBounds(skel, mot)
%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%COMPUTEBOUNDS Compute lower and upper bounds on joint angles from motion data.
%   [lb,ub] = computeBounds(skel, mot)
%
%   Note: the limits in the ASF file are only really used for editing
%   software. The solved motion violates the limits regularly it seems. So
%   here we estimate limits from existing motion.
%
%INPUTS: 
%  skel -  The skeleton data structure following HDM05 sepecification.
%  mot - Previously solved motion from the HDM05 database.
%OUTPUTS: 
%  lb - Lower limits on degrees of freedom.
%  ub - Upper limits on degrees of freedom.
%
[ndof,dofMap] = computeDOF(skel);   % compute mapping from joints -> dofs
conversion_factor = pi/180.;    % degrees -> radians

%% Initialize the bounds.
%
lb = -1e20*ones(ndof,1);    % default bounds are esssentially [-inf,+inf]
ub = 1e20*ones(ndof,1);

%% Loop over all frames of motion and compute upper/lower bounds
for i = 2:skel.njoints % skip root joint since we don't impose limits on it
    sz = dofMap(i,2);
    if( sz > 0 )
        k = dofMap(i,1);
        kk = dofMap(i,1)+sz-1;
        lb(k:kk,:) = conversion_factor * min(mot.rotationEuler{i},[],2);
        ub(k:kk,:) = conversion_factor * max(mot.rotationEuler{i},[],2);
    end
end

%% Give some 'wiggle' room, and avoid equal lower/upper boundary values.
for i = 1:ndof
    lb(i) = lb(i) - 0.05*abs(lb(i));
    ub(i) = ub(i) + 0.05*abs(ub(i));
end

end