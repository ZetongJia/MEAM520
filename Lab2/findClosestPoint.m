function [qClosest] = findClosestPoint(qSample, nodes)
    k = keys(nodes);
    closestDistance = 1000;
	for i = 1:length(nodes)
        currDistance = distance(qSample, nodes(k{i}).q);
        if currDistance < closestDistance
            closestDistance = currDistance;
            qClosest = k{i};
        end
	end
end