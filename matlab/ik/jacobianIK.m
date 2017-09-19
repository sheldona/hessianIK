function [ J ] = jacobianIK(skel, bonePos, boneQuat, x, targets)
%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%JACOBIANIK Compute the Jacobian of the IK objective function.
%
% INPUT:  
%   skel - The skeleton data structure following HDM05 sepecification
%   bonePos - Bone positions as 3xN matrix.
%   boneQuat - Bone rotations as 4xN matrix;
%   dof - Root translation and Euler rotations for all joints created using
%     the packDof function.
%   targets - C3D position targets.
% OUTPUT:
%   J - The Jacobian matrix (3NxM) 

[ndofs,dofMap] = computeDOF(skel);           % includes 3 translational dofs
ncoords = 3 * targets.ntargets;

J = zeros(ncoords, ndofs);

conversion_factor = 1.;
if( strcmp(skel.angleUnit, 'deg') )
    conversion_factor = pi / 180.;
end

% For each target ...
for i = 1:targets.ntargets
    row = 3*(i-1)+1;                % starting row index
    J(row:row+2,1:3) = eye(3);      % translational dofs have identity Jac.
    
    endBoneId = targets.dependsBones{i}(1);
    endEffectorPos = quatrot(targets.offset{i}, boneQuat(:,endBoneId)) + bonePos(:,endBoneId);
    
    % For each dependent joint ...
    for j = targets.dependsBones{i}
        q_axis = euler2quat(flipud(skel.nodes(j).axis)*conversion_factor,'zyx');
        
        col = dofMap(j,1);
        sz = dofMap(j,2);
        if( sz > 0 )
            parentID = skel.nodes(j).parentID;
            delta_p = endEffectorPos - bonePos(:,j);
            if( parentID == 0 )     % special case: the root bone
                idx0 = 4;                
            else
                idx0 = 1;
            end
            
            q_parent = [1;0;0;0];
            if( parentID > 0 )  % check if valid parent ID
                 q_parent = boneQuat(:,parentID);
             end

            q_accum = [1;0;0;0];
            for k = sz:-1:idx0 % reverse the order to get it right!
                dofId = lower(skel.nodes(j).DOF{k});
                q_alpha = [1;0;0;0];
                m = col+k-1;
                v = [];
                q = quatmult(q_axis,q_accum);
                if( strcmp(dofId,'rx') )
                    v = quatrot([1;0;0], quatmult(q_parent,q));
                    q_alpha = rotquat(x(m), 'x');
                elseif( strcmp(dofId,'ry') )
                    v = quatrot([0;1;0], quatmult(q_parent,q));
                    q_alpha = rotquat(x(m), 'y');
                elseif(  strcmp(dofId,'rz') )
                    v = quatrot([0;0;1], quatmult(q_parent,q));
                    q_alpha = rotquat(x(m), 'z');
                end
                q_accum = quatmult(q_accum,q_alpha);
                J(row:row+2,m) = cross(v, delta_p);
            end       
        end        
    end      
end

end

