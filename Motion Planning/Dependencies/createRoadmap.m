function [nodes, G] = createRoadmap(map, lim)
    [cellDecomp, mapLine] = cellDecomposition(map, lim);
    nodes = [cellDecomp(:, 1) (cellDecomp(:, 2) + cellDecomp(:, 4))/2];
    extMap = [mapLine; cellDecomp]; tempNodes = []; chkNodes = [];    
    for i=1:size(nodes, 1)
        dep = depthPredict([nodes(i, :) pi], extMap, [0 0], 0);
        tempNodes = [tempNodes; nodes(i, 1) - dep/2, nodes(i, 2)];
    end
    for i=1:size(tempNodes, 1)
        for j=i+1:size(tempNodes, 1)
            for k=1:size(extMap, 1)
                isect(k, 1) = intersectPoint(tempNodes(i,1),tempNodes(i,2),tempNodes(j,1),tempNodes(j,2),extMap(k, 1),extMap(k, 2),extMap(k, 3),extMap(k, 4));
            end
            if any(isect), continue; end
            chkNodes = [chkNodes; tempNodes(i,:); tempNodes(j, :)];
            isect = 0;
        end
    end
    [~, idx] = intersect(tempNodes, chkNodes, 'rows'); tempNodes(idx, :) = []; nodes = [nodes; tempNodes];
    while 1
        nodes = [nodes; mean(chkNodes(abs(chkNodes(:,1) - chkNodes(1, 1)) < 1e-3,:), 1)];
        chkNodes(abs(chkNodes(:,1) - chkNodes(1, 1)) < 1e-3,:) = [];
        if size(chkNodes, 1) == 0, break; end
    end
    [~, idx] = sort(nodes(:, 1)); nodes = nodes(idx, :); nodes(end + 1, :) = [(cellDecomp(end, 1) + lim(3))/2 (lim(2) + lim(4))/2];
    G = graph; G = addnode(G, size(nodes, 1));edge = [];
    for i = 1:size(nodes, 1) - 1
        for j = i + 1:size(nodes, 1)
            for k=1:size(extMap, 1)
                isect(k, 1) = intersectPoint(nodes(i, 1) + 1e-3, nodes(i, 2), nodes(j, 1) - 1e-3, nodes(j, 2), extMap(k, 1), extMap(k, 2), extMap(k, 3), extMap(k, 4));
            end
            if (~any(isect)), edge = [edge; i j];  end
        end
        if isempty(edge), continue; end
        if size(edge, 1) > 1, edge = edge(edge(:, 2) == find(nodes(:, 1) == nodes(edge(1, 2), 1)), :); end
        G = addedge(G, edge(:, 1), edge(:, 2), sum((nodes(edge(:, 1),:) - nodes(edge(:, 2),:)).^2, 2).^0.5);
        edge = [];
    end
end