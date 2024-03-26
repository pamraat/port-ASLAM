function [G, nodes] = visibilityPRM(map, n, robotRad, lim, init, goal)
    nTry = 300;
    for i = 1:size(map, 2)
        temp = map(any(map(:, i), 2), i); temp = reshape(temp, 2, length(temp)/2)';
        side = [temp(2:end, :); temp(1, :)] - temp; side = side./(sum(side.^2, 2)).^0.5;
        bi = (side - [side(2:end, :); side(1, :)]);
        in = inpolygon(temp(:, 1) + 0.01*bi(:, 1), temp(:, 2) + 0.01*bi(:, 2), temp(:, 1), temp(:, 2));
        bi = (1 - 2*in).*bi./(sum(bi.^2, 2)).^0.5; bi = reshape(bi', 2*size(bi, 1), 1);
        map(1:length(bi), i) = map(1:length(bi), i) + 2*robotRad*[bi(end-1:end); bi(1:end - 2)];
    end
    [~, map] = cellDecomposition(map, lim);
    itr = 0; nodes = unifSample(1, map); G = graph; G = addnode(G, 1);
    while itr < nTry && size(nodes, 1) < n
        q = unifSample(1, map); isFree = edgeFree(nodes, q, map);
        if sum(isFree) > 1
            tempNodes = nodes(isFree, :); tempInd = find(isFree); isNew = false(0, 1);
            for j = 1:size(tempNodes, 1)    
                for k = j + 1:size(tempNodes, 1)
                    if j == 1, isNew(k) = isempty(shortestpath(G, tempInd(j), tempInd(k)));
                    else, isNew(k) = isempty(shortestpath(G, tempInd(j), tempInd(k))) & isNew(k);end
                end
            end
            if any(isNew), isNew(1) = true; nodes = [nodes; q]; G = addedge(G, size(nodes, 1), tempInd(isNew), 1); end
        elseif sum(isFree) == 0
            nodes = [nodes; q]; G = addnode(G, 1);
        end
        itr = itr + 1;
    end
    if nargin == 6
        nodes = [nodes; init; goal]; G = addnode(G, 2);
        indDist = knnsearch(nodes(1:end-2, :), init, 'K', size(nodes, 1) - 2);
        for j = 1:size(nodes, 1) - 2
            if edgeFree(nodes(indDist(j), :), init, map), G = addedge(G, size(nodes, 1) - 1, indDist(j), 1); break; end
        end
        indDist = knnsearch(nodes(1:end-2, :), goal, 'K', size(nodes, 1) - 2);
        for j = 1:size(nodes, 1) - 2
            if edgeFree(nodes(indDist(j), :), goal, map), G = addedge(G, size(nodes, 1), indDist(j), 1); break; end
        end
    end
end