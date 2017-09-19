%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
% A simple IK test script.
% 
clear all;
close all;

% Create a dummy skeleton
skel = emptySkeleton;
skel.angleUnit = 'rad';
skel.njoints = 3;
skel.animated = [1,2,3];

% root
root = emptySkeletonNode;
root.boneName = 'root';
root.rotationOrder = 'XYZ';
root.ID = 1;
root.parentID = 0;
root.children = 2;
root.DOF = {'TX';'TY';'TZ';'RX';'RY';'RZ'};

% bone0
bone0 = emptySkeletonNode;
bone0.boneName = 'bone0';
bone0.rotationOrder = 'XYZ';
bone0.ID = 2;
bone0.parentID = 1;
bone0.children = 3;
bone0.offset = [4;0;0];
bone0.DOF = {'rx';'ry';'rz'};

% bone1
bone1 = emptySkeletonNode;
bone1.boneName = 'bone0';
bone1.rotationOrder = 'XYZ';
bone1.ID = 3;
bone1.parentID = 2;
bone1.children = [];
bone1.offset = [4;0;0];
bone1.DOF = {'rx';'ry';'rz'};

skel.nodes = root;
skel.nodes(2) = bone0;
skel.nodes(3) = bone1;

% Create an empty motion frame.
oneFrame = emptyMotion;
oneFrame.njoints = skel.njoints;
oneFrame.nframes = 1;
oneFrame.animated = skel.animated;
oneFrame.unanimated = skel.unanimated;
oneFrame.angleUnit = 'rad';

ndof = computeDOF(skel);
x = zeros(ndof,1);
x(1:6) = [0;0;0;0;0;0]; % root
x(7:9) = [0;0;pi/2]; % bone0
x(10:12) = [0;0;0]; % bone1
oneFrame = unpackDOF(x, skel, oneFrame, 1);
oneFrame = convert2quat(skel,oneFrame);
[oneFrame.jointTrajectories,oneFrame.jointRotations] = forwardKinematicsQuat(skel,oneFrame);

% Create targets
targets = emptyTargets;        
targets.ntargets = 1;
targets.boneIds = [ 3 ];
targets.c3dPos{1} = [4;4;0];
targets.offset{1} = [0;0;0];
targets.dependsBones{1} = [3 2 1];

x0 = zeros(ndof,1);
[ bonePos, boneQuat ] = extractBonePosQuat( skel, oneFrame, 1 );
[ J ] = jacobianIK(skel, bonePos, boneQuat, x, targets);
[ H ] = hessianIK(skel, bonePos, boneQuat, x, targets, J);

method = 0;
useHessian = true;
useBounds = false;
[x,f] = runIK(x0, skel, targets, method, useHessian, useBounds);
oneFrame = unpackDOF(x, skel, oneFrame, 1);
oneFrame = convert2quat(skel,oneFrame);
[oneFrame.jointTrajectories,oneFrame.jointRotations] = forwardKinematicsQuat(skel,oneFrame);


