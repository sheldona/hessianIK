function [ skel, mot ] = centimeterSkelMot( skel, mot )
%CENTIMETERSKELMOT Convert ASF skeleton and motion to centimeters.
%
% Copyright (C) 2017    Sheldon Andrews
%
% Permission to use and modify in any way, and for any purpose, this
% software, is granted by the author.  Permission to redistribute
% unmodified copies is also granted.  Modified copies may only be
% redistributed with the express written consent of:
%   Sheldon Andrews (sheldon.andrews@gmail.com)
%
inches_to_centimeters = 2.5189;

for i = 1:skel.njoints,
    skel.nodes(i).offset = inches_to_centimeters * skel.nodes(i).offset;
    skel.nodes(i).length = inches_to_centimeters * skel.nodes(i).length;
    mot.jointTrajectories{i} = inches_to_centimeters * mot.jointTrajectories{i};
end

mot.rootTranslation = inches_to_centimeters * mot.rootTranslation;
mot.boundingBox = inches_to_centimeters * mot.boundingBox;

end

