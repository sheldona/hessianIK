%
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
% Please see runIK.m to further tune optimization parameters.

maxIter = 100;      % Max iterations of the quasi-Newton or exact method.
tol = 1e-50;        % Stopping tolerance for the objective function.
                     %  Setting this will run until maxIter is reached.
perturb = 0.0;      % Mean of the noise added to joint angles (TEST)
useLimits = false;  % Enable/disable solving with joint limits
useZeroInit = false;    % If true, initial solution is zero

%% Load input skeleton and motion
%
[skel,mot]       = readMocap('HDM05-Parser/data/HDM_dg.asf', 'HDM05-Parser/data/HDM_dg_03-02_01_120.amc');
[skelC3D,motC3D] = readMocap('HDM05-Parser/data/HDM_dg_03-02_01_120.c3d', [], false);

%% Post-processing motion data for our IK solver.
%
[skel,mot] = centimeterSkelMot(skel,mot);   % Scale ASF/AMC motion from inches->centimetres
[skelC3D,motC3D] = removeMarkers012(skelC3D,motC3D);    % Remove markers *0,*1,*2
conversion_factor = pi / 180.;

%% Frames selected for the type of motion.
%
fastKickIndices = [489,2469,2842];       % kicking
fastPunchIndices = [3612,3624,4005];     % punching
slowMoIndices = [287,321,3375,6538];     % walking and standing
% Choose an index set from above.
indices = slowMoIndices;

%% Create targets.
%
targets = createTargets(skel, mot, motC3D);

%% Compute joint limits.
%
[skel.lb,skel.ub] = computeBounds(skel, mot);

%% Create history struct to store convergence details.
% 
nframes = size(indices,2);
Hist_exact = cell(nframes,1);
Hist_quasi = cell(nframes,1);
Hist_lm = cell(nframes,1);
Hist_idx = 1;

ndof = computeDOF(skel);
x0 = zeros(ndof,1);
%%
% Loop over selected frames and solve.
for k = indices
    % Store targets (optical marker positions from C3D)
    Pos = zeros(3,targets.ntargets);
    for i = 1:targets.ntargets
        boneId = targets.boneIds(i);
        qinv = quatinv(mot.jointRotations{boneId}(:,k));
        targets.offset{i} = quatrot(motC3D.jointTrajectories{i}(:,k) - mot.jointTrajectories{boneId}(:,k), qinv);
        targets.c3dPos{i} = motC3D.jointTrajectories{i}(:,k);
    end
    
    if( useZeroInit )
        % Initialize joint angles with zero, root pos with average marker pos 
        x0(1:3) = mean(cell2mat(targets.c3dPos),2);
        x0(4:end) = zeros(ndof-3,1) + perturb * (rand(ndof-3,1) - 0.5);
    else
        % Initialize with the previous frame of motion
        x0 = packDOF(skel,mot,k-1);
        x0(4:end) = conversion_factor * x0(4:end) + perturb * (rand(ndof-3,1) - 0.5);
    end

    % Run a solve with the exact Hessian
    tic;
    [x,f,hist] = runIK(x0, skel, targets, 0, true, useLimits, tol, maxIter);
    hist.t = toc;
    Hist_exact{Hist_idx} = hist;
       
    % Run a solve without the Hessian (quasi-Newton)
    tic;
    [x,f,hist] = runIK(x0, skel, targets, 1, false, useLimits, tol, maxIter);
    hist.t = toc;
    Hist_quasi{Hist_idx} = hist;
    
    if( ~useLimits),
        % Run a solve with LM, approximate Hessian
        tic;
        [x,f,hist] = runIK(x0, skel, targets, 2, false, useLimits, tol, maxIter);
        hist.t = toc;
        Hist_lm{Hist_idx} = hist;
    end

    Hist_idx = Hist_idx + 1;    % increment hist index
end

%
% Convergence plots.
%
lineStyles = { '-'; '--'; ':'; '-.' };
legendLbl = {};
legendIdx = 1;
figure;
for i = 1:nframes
    semilogy(Hist_exact{i}.fval', 'Color', 'black', 'LineStyle', lineStyles{i});
    if( i == 1)
        hold on;
    end   
    legendLbl{legendIdx} = [ 'Exact Hessian - Frame ' num2str(indices(i)) ];
    legendIdx = legendIdx + 1;
end
for i = 1:nframes
    h = semilogy(Hist_quasi{i}.fval', 'r-', 'LineStyle', lineStyles{i});
    legendLbl{legendIdx} = [ 'quasi-Newton - Frame ' num2str(indices(i)) ];
    legendIdx = legendIdx + 1;
end
if( ~useLimits )
    for i = 1:nframes
        h = semilogy(Hist_lm{i}.fval', 'b-', 'LineStyle', lineStyles{i});
        legendLbl{legendIdx} = [ 'Levenberg-Marq. - Frame ' num2str(indices(i)) ];
        legendIdx = legendIdx + 1;
    end
end

hold off;
h = legend(legendLbl);
set(h,'Interpreter','latex');
xlabel('Iterations');
ylabel('f(\theta)');
xlim([1 maxIter]);
ylim([1e-25 1e4])

%% Output some timing information
%
Data_hess = struct2dataset(cell2mat(Hist_exact(:)));
Data_quasi = struct2dataset(cell2mat(Hist_quasi(:)));
disp(['Hessian optimization took ' num2str(mean(Data_hess.t)) ' s, ' num2str(mean(Data_hess.funccount)) ' funcs.']);
disp(['Quasi-Newton optimization took ' num2str(mean(Data_quasi.t)) ' s, ' num2str(mean(Data_quasi.funccount)) ' funcs.']);

if( ~useLimits )
    Data_lm = struct2dataset(cell2mat(Hist_lm(:)));
    disp(['Levenberg-Marquardt optimization took ' num2str(mean(Data_lm.t)) ' s, ' num2str(mean(Data_lm.funccount)) ' funcs.']);
end
