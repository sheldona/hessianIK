% This code belongs to the HDM05 mocap database which can be obtained
% from the website http://www.mpi-inf.mpg.de/resources/HDM05 .
%
% If you use and publish results based on this code and data, please
% cite the following technical report:
%
%   @techreport{MuellerRCEKW07_HDM05-Docu,
%     author = {Meinard M{\"u}ller and Tido R{\"o}der and Michael Clausen and Bernd Eberhardt and Bj{\"o}rn Kr{\"u}ger and Andreas Weber},
%     title = {Documentation: Mocap Database {HDM05}},
%     institution = {Universit{\"a}t Bonn},
%     number = {CG-2007-2},
%     year = {2007}
%   }
%
%
% THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
% KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
% PARTICULAR PURPOSE.

function [jointTrajectories,jointRotations] = forwardKinematicsQuat(skel,mot)
[jointTrajectories,jointRotations] = recursive_forwardKinematicsQuat(skel,...
                                                mot,...
                                                1,...
                                                mot.rootTranslation + repmat(skel.nodes(1).offset,1,mot.nframes),...
                                                quatmult(repmat(skel.rootRotationalOffsetQuat,1,mot.nframes),mot.rotationQuat{1}),...
                                                mot.jointTrajectories, mot.jointRotations);