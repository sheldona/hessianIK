% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
% This script runs the IK solver for the HDM_dg_03-02_01_120 example in the
% HDM05 database. It also plots the convergence rate to compare the IK
% solver using BFGS vs. Trust region with exact Hessian.
% 
% Please see runIK.m to tune the optimization parameters.
%

maxIter = 10;       % Max iterations of the quasi-Newton or exact method.
tol = 1e-2;         % Stopping tolerance for the objective function.
useLimits = false;  % Enable/disable using limits
method = 0;         % 0 = exact Hessian, 1 = quasi-Newton, 2 = Levenberg–Marquardt

% Load input skeleton and motion
[skel,mot]       = readMocap('HDM05-Parser/data/HDM_dg.asf', 'HDM05-Parser/data/HDM_dg_03-05_02_120.amc');
[skelC3D,motC3D] = readMocap('HDM05-Parser/data/HDM_dg_03-05_02_120.c3d', [], false);

conversion_factor = pi / 180.;  % used to convert degrees -> radians
centimeters_to_inches = 1. / 2.5189;    % convert centimeters -> inches

%% Scale ASF/AMC motion from inches->centimetres
[skel,mot] = centimeterSkelMot(skel,mot);
[skelC3D, motC3D] = removeMarkers012(skelC3D, motC3D);

%% Create targets.
%
targets = createTargets(skel, mot, motC3D);

%% Compute joint limits.
%
[skel.lb,skel.ub] = computeBounds(skel, mot);

%% Pack the dofs for the first frame of motion. 
% 
dof = packDOF(skel,mot,1);
dof(4:end) = conversion_factor * dof(4:end);
ndof = size(dof,1);
x0 = dof;

mot_solved = mot;
mot_solved.angleUnit = 'deg';

%% Reconstruct each frame of motion using C3D positions
%
Hist = cell(mot.nframes,1);
for k = 1:mot.nframes
    disp(['-- frame ' num2str(k) ' of ' num2str(mot.nframes) ' --------------------------------']);
    
    % Store target positions
    for i = 1:targets.ntargets
        boneId = targets.boneIds(i);
        qinv = quatinv(mot.jointRotations{boneId}(:,k));
        targets.offset{i} = quatrot(motC3D.jointTrajectories{i}(:,k) - mot.jointTrajectories{boneId}(:,k), qinv);
        targets.c3dPos{i} = motC3D.jointTrajectories{i}(:,k);
    end

    % Run a solve with the Hessian
    tic;
    [x,f,hist] = runIK(x0, skel, targets, method, useLimits, tol, maxIter);
    hist.t = toc;
    Hist{k} = hist;
    
    x_amc = x;
    x_amc(1:3) = centimeters_to_inches * x(1:3);
    x_amc(4:end) = (1/conversion_factor) * x(4:end);
    mot_solved = unpackDOF(x_amc, skel, mot_solved, k);
    x0 = x;
end
disp('Done.');

Data = struct2dataset(cell2mat(Hisat(:)));
disp(['IK solve took ' num2str(mean(Data.t)) ' s per frame.']);

%% Update the solved motion with new bone positions and orientations. 
%
mot_solved = convert2quat(skel,mot_solved);
[mot_solved.jointTrajectories,mot_solved.jointRotations] = forwardKinematicsQuat(skel,mot_solved);

%% Reconstruct marker positions.
%
motC3D_solved = motC3D;
for i = 1:motC3D.njoints
    boneId = targets.boneIds(i);
    for k = 1:motC3D.nframes
        qinv = quatinv(mot.jointRotations{boneId}(:,k));
        targets.offset{i} = quatrot(motC3D.jointTrajectories{i}(:,k) - mot.jointTrajectories{boneId}(:,k), qinv);    
        motC3D_solved.jointTrajectories{i}(:,k) = ...
            mot_solved.jointTrajectories{boneId}(:,k) + quatrot(targets.offset{i}, mot_solved.jointRotations{boneId}(:,k));
    end
end

%% Uncomment line below to save the reconstructed motion
%writeAMC(skel,mot_solved,'HDM_dg_03-05_02_120_solved.amc');