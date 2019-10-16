function [q_closest] = findClosestPoint(q1, nodeMap)
    k = keys(nodeMap);
    closest_distance = 1000;
	for i = 1:length(nodeMap)
        distance = distance(q1,nodeMap(k{i}));
        if distance < closest_distance
            closest_distance = distance;
            q_closest = k{i};
        end
	end
end