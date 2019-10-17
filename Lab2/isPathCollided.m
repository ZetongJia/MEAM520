function [isPathCollided] = isPathCollided(qSample, qClosest, map, robot, numSteps)
    qSteps = qSample + [0:1/numSteps:1]' * (qClosest - qSample);
    isPathCollided = false;
    for i = 1:numSteps
        qStep = qSteps(i, :);
        isPathCollided = isPathCollided || isRobotCollided(qStep, map, robot);
    end
end