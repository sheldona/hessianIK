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

%% Load input skeleton and motion
%
[skel,mot]       = readMocap('HDM05-Parser/data/HDM_dg.asf', 'HDM05-Parser/data/HDM_dg_03-02_01_120.amc');
[skelC3D,motC3D] = readMocap('HDM05-Parser/data/HDM_dg_03-02_01_120.c3d', [], false);

%% Post-processing motion data for our IK solver.
%
[skel,mot] = centimeterSkelMot(skel,mot);   % Scale ASF/AMC motion from inches->centimetres
[skelC3D,motC3D] = removeMarkers012(skelC3D,motC3D);    % Remove markers *0,*1,*2
conversion_factor = pi / 180.;

%% Frames selected for the sweep.
%
indices = [3624];

%% Create targets.
%
targets = createTargets(skel, mot, motC3D);

%% Compute joint limits.
%
[skel.lb,skel.ub] = computeBounds(skel, mot);

%% Create history struct to store converge details.
% 
nframes = size(indices,2);
Hist = cell(nframes,1);
Hist_idx = 1;

%%
% Loop over selected frames and solve.
maxIter = 100;
tol = 1e-120;
for tolPCG = [0.01,0.1,0.2] % sweep PCG stopping tolerance
    for bPCG = [0,2,4,8,16,24]  % sweep PCG preconditioner bandwidth
        for k = indices
            % Store targets (optical marker positions from C3D)
            for i = 1:targets.ntargets
                boneId = targets.boneIds(i);
                qinv = quatinv(mot.jointRotations{boneId}(:,k));
                targets.offset{i} = quatrot(motC3D.jointTrajectories{i}(:,k) - mot.jointTrajectories{boneId}(:,k), qinv);
                targets.c3dPos{i} = motC3D.jointTrajectories{i}(:,k);
            end
            
            % Initialize with a nearby frame of motion
            dof = packDOF(skel,mot,k-1);
            dof(4:end) = conversion_factor * dof(4:end);
            x0 = dof;
            
            % Run a solve with the exact Hessian.
            % Note that here w  e set PCG tolerance and preconditioner
            % bandwidth.
            tic;
            [x,f,hist] = runIK(x0, skel, targets, 0, true, false, tol, maxIter, tolPCG, bPCG);
            hist.t = toc;
            hist.tolPCG = tolPCG;
            hist.bPCG = bPCG;
            Hist{Hist_idx,1} = hist;
            Hist_idx = Hist_idx + 1;
        end
    end
end

%% Convergence plots.
%
nhist = size(Hist,1);
lineStyles = { '-'; '--'; ':'; '-.' };
legendLbl = cell(nhist,1);
legendIdx = 1;
colors = gray(nhist+8);    % Create color map.
figure;
for i = 1:nhist
    semilogy(Hist{i}.fval', 'Color', colors(i,:), 'LineStyle', lineStyles{mod(i,4)+1});
    if( i == 1)
        hold on;
    end   
    legendLbl{legendIdx} = [ '$\epsilon_{PCG}$ = ' num2str(Hist{i}.tolPCG) ', $b_{PCG}$ = ' num2str(Hist{i}.bPCG) ];
    legendIdx = legendIdx + 1;
end
hold off;
h = legend(legendLbl);
set(h,'Interpreter','latex');
xlabel('Iterations');
ylabel('f(\theta)');
xlim([1 maxIter]);
ylim([1e-26 1e4])

