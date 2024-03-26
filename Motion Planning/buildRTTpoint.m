function [wayPoints, G, nodes] = buildRTTpoint(init, goal, step, map, lim)
    [~, map] = cellDecomposition(map, lim);
    nodes = init;
    G = graph; G = addnode(G, 1);
    while ~edgeFree(nodes(end, :), goal, map)
        isFree = 0;
        while ~isFree
            u = [(lim(3) - lim(1))*rand(1) + lim(1), (lim(4) - lim(2))*rand(1) + lim(2)];
            isFree = pointFree(u, map);
        end
        indNear = dsearchn(nodes, u);
        chkNode = step*(u - nodes(indNear, :))/norm(u - nodes(indNear, :)) + nodes(indNear, :);
        isFree = edgeFree(nodes(indNear, :), chkNode, map);
        if isFree
            nodes(end + 1, :) = chkNode; 
            G = addedge(G, indNear, size(nodes, 1), 1);
        end
    end
    nodes(end + 1, :) = goal; 
    G = addedge(G, size(nodes, 1) - 1, size(nodes, 1), 1);
    wayPoints = nodes(shortestpath(G, 1, size(nodes, 1)), :);
end