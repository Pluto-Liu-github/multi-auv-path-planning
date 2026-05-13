function weight_new = apply_current_drift_3d(weight, current_field, activity_quanju, drift_scale, fixed_ids)
%APPLY_CURRENT_DRIFT_3D Apply a small physical drift caused by 3D current.

if nargin < 4
    drift_scale = 0.45;
end
if nargin < 5
    fixed_ids = [];
end

weight_new = weight;
[MAX_X, MAX_Y, MAX_Z] = size(activity_quanju);

for j = 1:size(weight, 2)
    node_id = weight(1, j);
    if ismember(node_id, fixed_ids)
        continue;
    end

    pos = weight(2:4, j);
    grid_pos = max([1; 1; 1], min([MAX_X; MAX_Y; MAX_Z], round(pos)));
    current_vec = sample_current_3d(current_field, grid_pos(1), grid_pos(2), grid_pos(3));
    target_pos = pos + drift_scale * current_vec;
    target_pos = max([1; 1; 1], min([MAX_X; MAX_Y; MAX_Z], target_pos));

    grid_target = max([1; 1; 1], min([MAX_X; MAX_Y; MAX_Z], round(target_pos)));
    if activity_quanju(grid_target(1), grid_target(2), grid_target(3)) ~= -1
        weight_new(2:4, j) = target_pos;
    end
end
end
