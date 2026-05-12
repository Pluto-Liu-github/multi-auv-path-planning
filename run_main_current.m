% Project entry point for the 3D current-aware simulation.
project_root = fileparts(mfilename('fullpath'));
addpath(fullfile(project_root, 'matlab', 'core'));
addpath(fullfile(project_root, 'matlab', 'baselines'));
addpath(fullfile(project_root, 'matlab', 'utils'));
addpath(fullfile(project_root, 'matlab', 'visualization'));
addpath(fullfile(project_root, 'matlab', 'current'));
run(fullfile(project_root, 'mainin_3d_current.m'));
