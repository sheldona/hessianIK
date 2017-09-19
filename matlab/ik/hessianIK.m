function [ H ] = hessianIK(skel, bonePos, boneQuat, x, targets, J)
%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%HESSIANIK Compute the Hessian of the IK objective function.
%
% INPUT:  
%   skel - The skeleton data structure following HDM05 sepecification
%   bonePos - Bone positions as 3xN matrix.
%   boneQuat - Bone rotations as 4xN matrix;
%   dof - Root translation and Euler rotations for all joints created using
%     the packDof function.
%   targets - C3D position targets.
%   J - The Jacobian.
% OUTPUT:
%   H - The Hessian.

[ndofs,dofMap] = computeDOF(skel);  % Note that dofMap includes 3 translational dofs

% Initialize the Hessian.
H = J'*J;

conversion = 1;
if( strcmp(skel.angleUnit, 'deg') ),
    conversion = pi / 180;
end

% For each target ...
for t = 1:targets.ntargets
    row = 3*(t-1)+1;        % starting row index
    
    endBoneId = targets.dependsBones{t}(1);
    endEffectorPos = quatrot(targets.offset{t}, boneQuat(:,endBoneId)) + bonePos(:,endBoneId);
    r = targets.c3dPos{t} - endEffectorPos;
    
    % Outer joint loop
    for k = targets.dependsBones{t}
        col_k = dofMap(k,1);
        sz_k = dofMap(k,2);
        if( k == 1 )       % special case of root bone
            idx_k = 4;                
        else
            idx_k = 1;
        end                
        
        % Inner joint loop
        for h = targets.dependsBones{t}
            col_h = dofMap(h,1);
            sz_h = dofMap(h,2);
            if( h == 1 )      % special case of root bone
                idx_h = 4;                
            else
                idx_h = 1;
            end             
            
            dH = zeros(sz_h,sz_k);  % Initialize modification to Hessian
            
            if( col_h <= col_k )       % only want lower triangle part

                % Compute axis offset and the inverse.
                q_axis = euler2quat(flipud(skel.nodes(h).axis)*conversion,'zyx');         
               
                for i = sz_k:-1:idx_k  % Loop over joint k dofs
                   
                    kk = col_k+i-1;
                    J_kk = J(row:row+2,kk);
                    
                    parent_h = skel.nodes(h).parentID;
                    q_parent = [1;0;0;0];
                    if( parent_h > 0 )
                         q_parent = boneQuat(:,parent_h);
                     end

                    q_accum = [1;0;0;0];                    
                    for j = sz_h:-1:idx_h      % Loop over joint h dofs
                        dofId = lower(skel.nodes(h).DOF{j});
                        q_alpha = [1;0;0;0];
                        m = col_h+j-1;
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
                        
                        % Hessian is instantaneous joint(h) axis crossed
                        % with instantaneous joint(k) axis projected onto
                        % residual vector.
                        dH(j,i) = dot(cross(v, J_kk),r);
                    end
                end
                
                H(col_h:col_h+sz_h-1,col_k:col_k+sz_k-1) = H(col_h:col_h+sz_h-1,col_k:col_k+sz_k-1) - dH;
                if( h ~= k )
                    H(col_k:col_k+sz_k-1,col_h:col_h+sz_h-1) = H(col_k:col_k+sz_k-1,col_h:col_h+sz_h-1) - dH';
                end
                
            end
        end
    end

end

