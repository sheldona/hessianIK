function [ skelC3D, motC3D ] = removeMarkers012( skelC3D, motC3D )
%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
%REMOVEMARKERS012 Remove markers *0, *1, *2 from the C3D struct.

% Remove first three markers.
jointTrajectories = cell(motC3D.njoints-3,1);
nameMap = cell(motC3D.njoints-3,3);
for i = 4:motC3D.njoints
	jointTrajectories{i-3,1} = motC3D.jointTrajectories{i,1};
    nameMap{i-3,1} = motC3D.nameMap{i,1};
    nameMap{i-3,2} = 0;
    nameMap{i-3,3} = motC3D.nameMap{i,3}-3;
end
motC3D.jointTrajectories = jointTrajectories;
motC3D.nameMap = nameMap;
motC3D.njoints = motC3D.njoints - 3;

skelC3D.njoints = skelC3D.njoints - 3;
skelC3D.nameMap = nameMap;
skelC3D.paths = cell(skelC3D.njoints,1);
for i = 1:skelC3D.njoints
    skelC3D.paths{i,1} = [i;i];
end

end

