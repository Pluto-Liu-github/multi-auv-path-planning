function [dist_act, dist_total, energy_total] = A1_somdist_act_current_3d(Weight_act_total, w, dist_act_1, energy_1, V_AUV, current_field)
%A1_SOMDIST_ACT_CURRENT_3D Estimate 3D travel distance and current-aware energy.

u_1 = Weight_act_total{w}(2:4, :);
u_2 = Weight_act_total{w + 1}(2:4, :);
N = size(u_2, 2);
energy = zeros(1, N);

for t = 1:N
    move_vec = u_2(:, t) - u_1(:, t);
    disto = norm(move_vec);
    if disto == 0
        energy(t) = 0;
        continue;
    end

    current_vec = sample_current_3d(current_field, u_1(1, t), u_1(2, t), u_1(3, t));
    move_unit = move_vec / disto;
    current_along = dot(current_vec, move_unit);

    ground_speed = max(0.15 * V_AUV, V_AUV + current_along);
    travel_time = disto / ground_speed;
    relative_effort = max(0.2, 1 - current_along / max(V_AUV, eps));

    energy(t) = (V_AUV^3) * travel_time * relative_effort;
end

dist_act = sum(sqrt(sum((u_2 - u_1).^2, 1)));
dist_total = dist_act + dist_act_1;
energy_total = sum(energy) + energy_1;
end
