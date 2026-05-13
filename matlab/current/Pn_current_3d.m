function Pn_point = Pn_current_3d(huosheng_1_1, activity_3_1, ii, MAX_X, MAX_Y, MAX_Z, current_field, current_gain)
%PN_CURRENT_3D Select next 3D node using activity plus current alignment.

if nargin < 8
    current_gain = 0.005;
end

nodeId = huosheng_1_1(1, ii);
x0 = huosheng_1_1(2, ii);
y0 = huosheng_1_1(3, ii);
z0 = huosheng_1_1(4, ii);

current_vec = sample_current_3d(current_field, x0, y0, z0);
current_norm = norm(current_vec);

candidateNodes = [];
candidateActivity = [];
candidateAlignment = [];

for dx = -1:1
    for dy = -1:1
        for dz = -1:1
            if dx == 0 && dy == 0 && dz == 0
                continue;
            end

            x = x0 + dx;
            y = y0 + dy;
            z = z0 + dz;
            if x < 1 || x > MAX_X || y < 1 || y > MAX_Y || z < 1 || z > MAX_Z
                continue;
            end

            activity_score = activity_3_1(x, y, z);
            if activity_score < 0
                continue;
            end

            step_vec = [dx; dy; dz];
            step_norm = norm(step_vec);
            if current_norm > 0 && step_norm > 0
                alignment = dot(step_vec / step_norm, current_vec / current_norm);
            else
                alignment = 0;
            end

            candidateNodes(:, end + 1) = [x; y; z]; %#ok<AGROW>
            candidateActivity(end + 1) = activity_score; %#ok<AGROW>
            candidateAlignment(end + 1) = alignment; %#ok<AGROW>
        end
    end
end

if isempty(candidateActivity)
    bestNode = [x0; y0; z0];
    bestScore = activity_3_1(x0, y0, z0);
    Pn_point = [nodeId; bestNode; bestScore];
    return;
end

bestActivity = max(candidateActivity);
activityTolerance = 0.005;
nearBestMask = candidateActivity >= bestActivity - activityTolerance;
currentBonus = zeros(size(candidateActivity));
currentBonus(nearBestMask) = current_gain * candidateAlignment(nearBestMask) * min(current_norm, 1);
score = candidateActivity + currentBonus;

[~, bestIdx] = max(score);
bestNode = candidateNodes(:, bestIdx);
bestScore = candidateActivity(bestIdx);

Pn_point = [nodeId; bestNode; bestScore];
end
