% Add categorized MATLAB source folders to the path when MATLAB starts here.
% The archived 2D folders under matlab/original_2d are intentionally not
% added here because they contain functions with the same names as the
% current 3D code.
project_root = fileparts(mfilename('fullpath'));
matlab_runtime_dirs = {
    fullfile(project_root, 'matlab', 'core')
    fullfile(project_root, 'matlab', 'baselines')
    fullfile(project_root, 'matlab', 'utils')
    fullfile(project_root, 'matlab', 'visualization')
};

for k = 1:numel(matlab_runtime_dirs)
    if exist(matlab_runtime_dirs{k}, 'dir')
        addpath(matlab_runtime_dirs{k});
    end
end
