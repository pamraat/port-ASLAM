function free = edgeFree(points, goal, map)
    free = false(size(points, 1), 1); isect = zeros(size(map, 1), 1);
    for i = 1:size(points, 1)
        for k=1:size(map, 1)
            isect(k, 1) = intersectPoint(points(i, 1), points(i, 2), goal(1, 1), goal(1, 2), map(k, 1), map(k, 2), map(k, 3), map(k, 4));
        end
        free(i, 1) = logical(~any(isect));
    end
end