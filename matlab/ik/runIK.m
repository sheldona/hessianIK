function [x,f,history] = runIK(varargin) 
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%RUNIK Run an IK optimization. 
%   [x,f,history] = runIK(x0, skel, targets, method, useHessian, useBounds, ftol, maxiter, epsPCG, bPCG)
%INPUTS: 
%  x0 - Initial solution, the dofs of the skeleton including root trans.
%  skel -  The skeleton data structure following HDM05 sepecification.
%  targets - The position of C3D markers.
%  method - 0 = trust-region method and exact Hessian, 
%           1 = a quasi-Newton method (BFGS) with exact gradient
%           2 = Levenberg-Marquardt
%  useBounds - Apply joint angle box limits, in which case fmincon is used.
%  ftol - Stopping tolerance for the IK objective function. (Default 1e-20)
%  maxiter - Maximum number of iterations to use in the Newton or
%       quasi-Newton method.
%  epsPCG - Preconditioned Conjugate Gradient (PCG) stopping tolerance.
%       This is only used by the exact Newton algorithm with Hessian, which
%       is based on a reflective trust region method. (Default 0.1)
%  bPCG - Bandwidth of the preconditioner for PCG. (Default 24)
%OUTPUTS:
%  x - The optimal solution, which is the degrees of freedom of the
%       skeleton
%  f - Objective function value at the optimal solution.
%  history - Cell array storing information about each iteration. 
%
if( nargin < 3 )
    disp('Usage: [x,f,history] = runIK(x0, skel, targets, method, useHessian, useBounds, ftol, maxiter, epsPCG, bPCG)');
    return;
else
    x0 = varargin{1};
    skel = varargin{2};
    targets = varargin{3};
    method = 0;
    useHessian = false;
    useBounds = false; 
    ftol = 1e-20;
    maxiter = 100;
    epsPCG = 1e-1;   % Default values for epsPCG and bPCG selected based on 
    bPCG = 8;       %   parameter sweep (see doParamSweep.m)
end

if( nargin > 3 )
    method = varargin{4};
end
if( nargin > 4 )
    useHessian = varargin{5};
end
if( nargin > 5 )
    useBounds = varargin{6};
end
if( nargin > 6 )
    ftol = varargin{7};
end
if( nargin > 7 )
    maxiter = varargin{8};
end
if( nargin > 8 )
    epsPCG = varargin{9};
end
if( nargin > 9 )
    bPCG = varargin{10};
end

%% Main optimization code
%
ndof = size(x0,1);
conversion_factor = 1.;
if( strcmp(skel.angleUnit,'deg') )  % conversion: degrees-to-radians
    conversion_factor = pi/180.;
end

if( method < 2 )
    
    if( ~useBounds )    % setup unconstrained optimization
        options = optimoptions(@fminunc);
        if( useHessian )
            options.Algorithm = 'trust-region';
            options.HessianFcn = 'objective';
        else
            options.Algorithm = 'quasi-newton';
            options.HessianFcn = [];
            options.HessUpdate = 'bfgs';
        end 
    else  % setup constrained optimization with joint limits
        options = optimoptions(@fmincon);
        if( useHessian )
            options.Algorithm = 'trust-region-reflective';
            options.HessianFcn = 'objective';
        else
            options.Algorithm = 'interior-point';
            options.HessianFcn = [];
        end 
    end
    %
    % Options common to MATLAB algorithms
    %
    options.OutputFcn = @outfun;      
    options.Display = 'iter';           
    options.StepTolerance = 1e-120;         

    % Trust region options
    % 
    options.MaxPCGIter = ndof*ndof;    % Give cg more time to solve sub-problem.
    options.TolPCG = epsPCG;            % Setting this too low will give poor convergence!
    options.PrecondBandWidth = bPCG;    % Close to diagonal pre-conditioner seems to give good performance.
    
    options.FunctionTolerance= 1e-120;
    options.SpecifyObjectiveGradient = true;    % We always specify the gradient.
    %options.CheckGradients = true;            % Uncomment to verify the gradient.
    options.OptimalityTolerance = 1e-120;
    options.MaxIterations = maxiter;    
    options.MaxFunctionEvaluations = 1e6;       % Don't limit function evals, just max iterations.
else
    
    % Use our custom LM solver
    %
    options = LMFsolve('default');
    options.Display = 1;
    options.MaxIter = maxiter;
    options.ScaleD = [];
    options.FunTol = 1e-120;
    options.XTol = 1e-120;
    options.ExactHessian = useHessian;
    options.HessianFcn = 'objective';
    options.OutputFcn = @outfun;
    options.SpecifyObjectiveGradient = true;
end

% Create struct to keep track of function values and 
% the solution at each iteration.
history.fval = [];
history.x = [];
history.funccount = 0;

% Create an empty motion struct to store the single frame 
% which is being optimized. We do this so that we can 'piggy back' on 
% the HDM05 forward kinematics functions.
oneFrame = emptyMotion;
oneFrame.njoints = skel.njoints;
oneFrame.nframes = 1;
oneFrame.jointNames = skel.jointNames;
oneFrame.nameMap = skel.nameMap;
oneFrame.animated = skel.animated;
oneFrame.unanimated = skel.unanimated;
oneFrame.angleUnit = 'rad';

if( method < 2 && ~useBounds )    % unconstrained optimization
    [x,f] = fminunc(@ikFunc,x0,options);
elseif( method < 2 )    % constrained optimization with joint limits
    [x,f] = fmincon(@ikFunc,x0,[],[],[],[],skel.lb,skel.ub,[],options);
else
    [x,f] = LMFsolve(@ikFunc,x0,options);    
end

%% Nested function to evaluate the IK solution x. 
% The exact gradient and Hessian are also returned.
%
function [f,g,H] = ikFunc(x)
    oneFrame = unpackDOF(x, skel, oneFrame, 1);
    oneFrame = convert2quat(skel,oneFrame);
    [oneFrame.jointTrajectories,oneFrame.jointRotations] = forwardKinematicsQuat(skel,oneFrame);
    [bonePos, boneQuat] = extractBonePosQuat(skel,oneFrame,1);
    [f, r] = objectiveIK(skel, bonePos, boneQuat, targets);    
    H = [];
    g = [];
    
    if( method < 2 )
        if( nargout > 1 && options.SpecifyObjectiveGradient )  % only compute gradient if req'd.
            [ J ] = jacobianIK(skel, bonePos, boneQuat, x, targets);   
            g = -J'*r;
        end
    else
        if( nargout > 1 && options.SpecifyObjectiveGradient )  % only compute gradient if req'd.
            [ J ] = jacobianIK(skel, bonePos, boneQuat, x, targets);   
            g = J;
        end        
        f = r;
    end
    
    if( nargout > 2 && ~isempty(options.HessianFcn) )	% only compute Hessian if req'd.
        [ H ] = hessianIK(skel, bonePos, boneQuat, x, targets, J);
%         H = nearestSPD(H);        
%         disp(['Condition number ' num2str(cond(H)) ', rank ' num2str(rank(H))]);
%         if( ~check )
%            y = 1; 
%         end
    end        
end

%% Nested function to collect optimization 
%
function stop = outfun(x,optimValues,state)
    stop = false;
       switch state
           case 'iter'
               % Concatenate current point and objective function
               % value with history. x must be a row vector.
               if( isfield(optimValues, 'fval') )
                   [m,n] = size(optimValues.fval);
                   if( m > 1 ),
                       fval = 0.5*optimValues.fval'*optimValues.fval;
                   else
                        fval = optimValues.fval;
                   end
               elseif( isfield(optimValues, 'residual') )
                   fval = 0.5*optimValues.residual'*optimValues.residual;
               end
               history.fval = [history.fval; fval];
               history.x = [history.x; x];
               history.funccount = history.funccount + optimValues.funccount;
               if( fval < ftol )
                   stop = true;
               end
           otherwise
       end
    end
        
end

