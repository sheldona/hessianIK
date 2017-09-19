function [ f, r ] = objectiveIK(skel, bonePos, boneQuat, targets)
%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%OBJECTIVEIK Compute the IK objective function.
%   f = r'*r, where r is the position error vector between the end effector
%   and the targets.
% INPUT: 
%   skel - The skeleton data structure following HDM05 sepecification
%   bonePos - Bone positions as 3xN matrix.
%   boneQuat - Bone rotations as 4xN matrix;
%   dof - Root translation and Euler rotations for all joints created using
%     the packDof function.
%   targets - C3D position targets.
% OUTPUT:
%   f - The value of the objective function.
%   r - The residual vector (3Nx1).
%

% Create residual vector
ncoords = 3 * targets.ntargets;
r = zeros(ncoords,1);

% For each target ...
for i = 1:targets.ntargets
    k = 3*(i-1)+1;
    endBoneId = targets.dependsBones{i}(1);
    endEffectorPos = quatrot(targets.offset{i}, boneQuat(:,endBoneId)) + bonePos(:,endBoneId);
    r(k:k+2) = targets.c3dPos{i} - endEffectorPos;
end

f = 0.5*(r'*r);

end

