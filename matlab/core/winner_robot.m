function huosheng_1 = winner_robot(Weight_vir, activity_dt, TargetNum, activity_quanju, close_count, robot_energy_total, gamma_penalty, robot_task_count, load_penalty, fixed_target_robot)
    huosheng_1 = zeros(6, TargetNum); % [机器人编号; X; Y; Z; 全局活性; 竞争得分]
    num_nodes = size(Weight_vir, 2);

    if nargin < 8 || isempty(robot_task_count)
        max_robot_id = max(Weight_vir(1, :));
        robot_task_count = zeros(1, max_robot_id);
    end
    if nargin < 9 || isempty(load_penalty)
        load_penalty = 0.45;
    end
    if nargin < 10 || isempty(fixed_target_robot)
        fixed_target_robot = zeros(1, TargetNum);
    end

    assigned_robots = [];

    for ii = 1:TargetNum
        if ismember(ii, close_count)
            continue;
        end

        fixed_robot = 0;
        if ii <= length(fixed_target_robot)
            fixed_robot = fixed_target_robot(ii);
        end

        [best_free_node, best_free_score, best_any_node, best_any_score] = choose_candidate( ...
            Weight_vir, activity_dt{ii}, activity_quanju, robot_energy_total, gamma_penalty, ...
            robot_task_count, load_penalty, assigned_robots, fixed_robot);

        if best_any_score == -inf && fixed_robot > 0
            [best_free_node, best_free_score, best_any_node, best_any_score] = choose_candidate( ...
                Weight_vir, activity_dt{ii}, activity_quanju, robot_energy_total, gamma_penalty, ...
                robot_task_count, load_penalty, assigned_robots, 0);
        end

        if best_free_score > -inf
            selected_node = best_free_node;
        else
            selected_node = best_any_node;
        end

        huosheng_1(:, ii) = selected_node;
        if selected_node(1) > 0
            assigned_robots = [assigned_robots, selected_node(1)];
        end
    end
end

function [best_free_node, best_free_score, best_any_node, best_any_score] = choose_candidate(Weight_vir, activity_map, activity_quanju, robot_energy_total, gamma_penalty, robot_task_count, load_penalty, assigned_robots, fixed_robot)
    best_any_score = -inf;
    best_any_node = zeros(6, 1);
    best_free_score = -inf;
    best_free_node = zeros(6, 1);

    for k = 1:size(Weight_vir, 2)
        robotID = Weight_vir(1, k);
        if fixed_robot > 0 && robotID ~= fixed_robot
            continue;
        end

        x = round(Weight_vir(2, k));
        y = round(Weight_vir(3, k));
        z = round(Weight_vir(4, k));

        x = max(1, min(size(activity_quanju, 1), x));
        y = max(1, min(size(activity_quanju, 2), y));
        z = max(1, min(size(activity_quanju, 3), z));

        base_act = activity_map(x, y, z);
        energy = 0;
        task_load = 0;
        if robotID > 0 && robotID <= length(robot_energy_total)
            energy = robot_energy_total(robotID);
        end
        if robotID > 0 && robotID <= length(robot_task_count)
            task_load = robot_task_count(robotID);
        end

        balance_factor = 1 + gamma_penalty * energy + load_penalty * task_load;
        if base_act > 0
            final_score = base_act / balance_factor;
        else
            final_score = base_act - gamma_penalty * energy - load_penalty * task_load;
        end

        global_act = activity_quanju(x, y, z);
        candidate_node = [robotID; x; y; z; global_act; final_score];

        if final_score > best_any_score
            best_any_score = final_score;
            best_any_node = candidate_node;
        end
        if ~ismember(robotID, assigned_robots) && final_score > best_free_score
            best_free_score = final_score;
            best_free_node = candidate_node;
        end
    end
end