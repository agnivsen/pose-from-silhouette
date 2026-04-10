
%% Description:
% This code generates a synthetic orthographic silhouette from the <Pelvic
% Bone> 3D model and estimates the the pose of the model from this input
% orthographic silhouette. The generated input orthographic silhouette is
% simulated at a random pose; the estimated pose and groundtruth pose are
% displayed at the end of each code execution. Depending upon the
% computational power, the code execution time may vary from a few seconds
% to a few minutes.

close all; clear all; clear vars; clc;

addpath(genpath('.'));

%% Getting shape signature details: 
shapeSignature = load('./Data/ShapeSignature.mat');
numPoseSteps = numel(shapeSignature.areaProfile);

%% Getting object shape: 
object = load('./Data/pointcloud.txt');
object = object.';

%% Generating a random pose: 
rotationEulerAngles = random_number_within_range(0,180,3);
R = rotx(rotationEulerAngles(1))*roty(rotationEulerAngles(2))*rotz(rotationEulerAngles(3));
t = random_number_within_range(-2,2,2); t = [t 0].';

%% Simulating orthographic silhouettes: 
objectRot = R*object + t;
plane.point = [0; 0; -3]; plane.normal = [0; 0; 1]; % An arbitrary plane parallel to XY
[~, orthProjInput] = OrthographicProjection(objectRot, plane);

%% Running GlOptiPoS:
fprintf('\n\nRunning <strong>GlOptiPoS</strong>.\n(This may take a few minutes)\n\n')
[objectEst] = GlOptiPoS(object, orthProjInput, shapeSignature);

visualizePose(objectRot, objectEst);
