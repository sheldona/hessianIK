function [ targets ] = emptyTargets( )
%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%EMPTYTARGETS Returns an empty target struct.

targets = struct('ntargets',0,...                               % number of targets
              'c3dNames',cell(1,1),...                          % names of c3d markers
              'c3dPos',cell(1,1),...                            % target positions in world coord space
              'boneIds',[],...                                  % vector of bone ids to c3d marker names
              'offset',cell(1,1),...                            % 3d offset of each marker in local bone coords
              'dependsBones',cell(1,1));                        % bone dependencies for this target
end

