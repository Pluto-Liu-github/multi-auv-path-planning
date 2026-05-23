function comparison = run_baseline_comparison(robotStarts, targets, obstacles, maxX, maxY, maxZ, vAuv, taskPairs)
%RUN_BASELINE_COMPARISON Compare A*, APF and GBSOM-compatible baselines.
    obstacleGrid = build_obstacle_grid(obstacles, maxX, maxY, maxZ);
    if nargin < 8 || isempty(taskPairs)
        taskPairs = nearest_robot_task_pairs(robotStarts, targets);
    else
        taskPairs = normalize_task_pairs(taskPairs, robotStarts, targets);
    end
    plannerNames = {'A*', 'APF'};
    allResults = cell(numel(plannerNames), size(taskPairs, 1));
    currentRobotPositions = cell(numel(plannerNames), 1);
    effectiveStarts = cell(numel(plannerNames), size(taskPairs, 1));

    for plannerIdx = 1:numel(plannerNames)
        currentRobotPositions{plannerIdx} = robotStarts;
    end

    for taskIdx = 1:size(taskPairs, 1)
        robotIdx = taskPairs(taskIdx, 1);
        goalNode = targets(:, taskPairs(taskIdx, 2));

        astarStart = currentRobotPositions{1}(:, robotIdx);
        effectiveStarts{1, taskIdx} = astarStart;
        astarResult = plan_astar_3d(astarStart, goalNode, obstacleGrid);
        astarResult.robotIndex = robotIdx;
        astarResult.targetIndex = taskPairs(taskIdx, 2);
        allResults{1, taskIdx} = astarResult;
        currentRobotPositions{1}(:, robotIdx) = next_robot_position(astarResult, goalNode);

        apfStart = currentRobotPositions{2}(:, robotIdx);
        effectiveStarts{2, taskIdx} = apfStart;
        apfResult = plan_apf_3d(apfStart, goalNode, obstacleGrid);
        apfResult.robotIndex = robotIdx;
        apfResult.targetIndex = taskPairs(taskIdx, 2);
        allResults{2, taskIdx} = apfResult;
        currentRobotPositions{2}(:, robotIdx) = next_robot_position(apfResult, goalNode);
    end

    comparison = summarize_planner_results(plannerNames, allResults, vAuv);
    comparison.taskPairs = taskPairs;
    comparison.effectiveStarts = effectiveStarts;
    comparison.obstacleGrid = obstacleGrid;
end

function position = next_robot_position(result, goalNode)
    if result.success
        position = goalNode(:);
    elseif ~isempty(result.path)
        position = result.path(end, :)';
    else
        position = result.startNode(:);
    end
end

function taskPairs = normalize_task_pairs(taskPairs, robotStarts, targets)
    if size(taskPairs, 2) ~= 2 && size(taskPairs, 1) == 2
        taskPairs = taskPairs';
    end

    robotCount = size(robotStarts, 2);
    targetCount = size(targets, 2);
    taskPairs = round(taskPairs(:, 1:2));
    validRows = taskPairs(:, 1) >= 1 & taskPairs(:, 1) <= robotCount & ...
        taskPairs(:, 2) >= 1 & taskPairs(:, 2) <= targetCount;
    taskPairs = taskPairs(validRows, :);

    cleanPairs = zeros(0, 2);
    usedTargets = [];
    for rowIdx = 1:size(taskPairs, 1)
        targetIdx = taskPairs(rowIdx, 2);
        if ismember(targetIdx, usedTargets)
            continue;
        end
        cleanPairs = [cleanPairs; taskPairs(rowIdx, :)];
        usedTargets = [usedTargets, targetIdx];
    end

    for targetIdx = 1:targetCount
        if ismember(targetIdx, usedTargets)
            continue;
        end
        distances = sqrt(sum((robotStarts - targets(:, targetIdx)).^2, 1));
        [~, robotIdx] = min(distances);
        cleanPairs = [cleanPairs; robotIdx, targetIdx];
    end

    taskPairs = cleanPairs;
end

function taskPairs = nearest_robot_task_pairs(robotStarts, targets)
    targetCount = size(targets, 2);
    taskPairs = zeros(targetCount, 2);

    for targetIdx = 1:targetCount
        target = targets(:, targetIdx);
        distances = sqrt(sum((robotStarts - target).^2, 1));
        [~, robotIdx] = min(distances);
        taskPairs(targetIdx, :) = [robotIdx, targetIdx];
    end
end

function comparison = summarize_planner_results(plannerNames, allResults, vAuv)
    comparison = struct();

    for plannerIdx = 1:numel(plannerNames)
        results = allResults(plannerIdx, :);
        success = cellfun(@(item) item.success, results);
        pathLengths = cellfun(@(item) item.pathLength, results);
        searchSteps = cellfun(@(item) item.searchSteps, results);
        validLengths = pathLengths(success);

        if isempty(validLengths)
            totalLength = inf;
            averageLength = inf;
            energy = inf;
        else
            totalLength = sum(validLengths);
            averageLength = mean(validLengths);
            energy = totalLength * vAuv^2;
        end

        fieldName = matlab.lang.makeValidName(plannerNames{plannerIdx});
        comparison.(fieldName) = struct( ...
            'method', plannerNames{plannerIdx}, ...
            'results', {results}, ...
            'successCount', sum(success), ...
            'taskCount', numel(results), ...
            'successRate', sum(success) / numel(results), ...
            'totalPathLength', totalLength, ...
            'averagePathLength', averageLength, ...
            'totalSearchSteps', sum(searchSteps), ...
            'equivalentEnergy', energy);
    end
end
