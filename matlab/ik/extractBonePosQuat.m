function [ bonePos, boneQuat ] = extractBonePosQuat( skel, mot, frame )
%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%EXTRACTBONEPOSQUAT Extract the bone positions and rotations (as
% quaternions) for a single frame of motion.
%
% INPUT:  
%   skel - The skeleton data structure following HDM05 sepecification.
%   mot - The motion data structure following HDM05 sepecification.
%   frame -  Index of the frame to extract.
% OUTPUT: 
%   bonePos - Bone positions as 3xN matrix.
%   boneQuat - Bone rotations as 4xN matrix.

bonePos = zeros(3,skel.njoints);
boneQuat = zeros(4,skel.njoints);
for i = 1:skel.njoints
    bonePos(:,i) = mot.jointTrajectories{i}(:,frame);
    boneQuat(:,i) = mot.jointRotations{i}(:,frame);
end

end

