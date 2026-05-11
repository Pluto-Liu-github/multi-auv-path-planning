function result = init_planner_result(method, startNode, goalNode)
%INIT_PLANNER_RESULT Create a common result struct for path planners.
    result = struct();
    result.method = method;
    result.startNode = startNode;
    result.goalNode = goalNode;
    result.path = [];
    result.pathLength = inf;
    result.searchSteps = 0;
    result.success = false;
    result.collision = true;
end
