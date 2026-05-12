% Project entry point. Run this file from the repository root.
project_root = fileparts(mfilename('fullpath'));
addpath(fullfile(project_root, 'matlab', 'core'));
addpath(fullfile(project_root, 'matlab', 'baselines'));
addpath(fullfile(project_root, 'matlab', 'utils'));
addpath(fullfile(project_root, 'matlab', 'visualization'));
run(fullfile(project_root, 'mainin.m'));
