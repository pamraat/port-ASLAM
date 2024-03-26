function [G, nodes] = buildPRM(map, n, f, robotRad, t, init, goal)
    [map, lim] = modLine(map, t, robotRad, 1); map = point2Line(map, lim);
    nodes = f(n , map, lim);
    if nargin == 7, nodes = [nodes; init; goal]; end
    G = graph; G = addnode(G, size(nodes, 1)); ind = 1:size(nodes, 1);
    for i = 1:size(nodes, 1)
        isFree = edgeFree(nodes(i + 1:end, :), nodes(i, :), map); isFree = [false(i, 1); isFree];
        G = addedge(G, i, ind(isFree), sum((nodes(ind(isFree), :) - nodes(i, :)).^2, 2).^0.5);
    end
end