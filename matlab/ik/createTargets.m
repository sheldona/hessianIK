function [ targets ] = createTargets(skel, mot, motC3D )
%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%CREATETARGETS Create tracking targets.
%
targets = emptyTargets;        
targets.ntargets = motC3D.njoints;
targets.c3dNames = { ...                % names of c3d markers
    'LFHD' 'RFHD' 'LBHD' 'RBHD' 'C7' 'T10' 'CLAV' 'STRN' 'RBAC' ...
    'LSHO' 'LUPA' 'LELB' 'LFRM' 'LWRA' 'LWRB' 'LFIN' ...
    'RSHO' 'RUPA' 'RELB' 'RFRM' 'RWRA' 'RWRB' 'RFIN' ...
    'LFWT' 'RFWT' 'LBWT' 'RBWT' ...
    'LTHI' 'LKNE' 'LSHN' 'LANK' 'LHEE' 'LTOE' 'LMT5' ...
    'RTHI' 'RKNE' 'RSHN' 'RANK' 'RHEE' 'RTOE' 'RMT5' };
targets.boneIds = [ ...                 % maps c3d markers to bone ids
    17 17 17 17 15 12 14 13 13 ...
    18 19 19 20 21 21 23 ...
    25 26 26 27 28 28 30 ...
    1 1 1 1 ...
    3 3 4 5 5 6 6 ...
    8 8 9 10 10 11 11 ];
% Find the offset of each marker and store in bone local coords.
for i = 1:targets.ntargets
    boneId = targets.boneIds(i);
    
    Offsets = zeros(3,mot.nframes);
    for j = 1:mot.nframes
        qinv = quatinv(mot.jointRotations{boneId}(:,j));
        Offsets(:,j) = quatrot(motC3D.jointTrajectories{i}(:,j) - mot.jointTrajectories{boneId}(:,j), qinv);
    end
    
    targets.offset{i} = mean(Offsets, 2);
    
    % Compute bone dependencies array.
    targets.dependsBones{i} = boneId;
    parentId = skel.nodes(boneId).parentID;
    while( parentId > 0 )
        targets.dependsBones{i} = [ targets.dependsBones{i} parentId ];
        parentId = skel.nodes(parentId).parentID;
    end
end

end

