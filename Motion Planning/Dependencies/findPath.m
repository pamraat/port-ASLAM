function [path, G, nodes] = findPath(init, goal, G, nodes, map)
    nodes = [nodes; init; goal];
    for i=1:2
        for j=1:size(nodes, 1) - 2
            for k=1:size(map, 1)
                isect(k, 1) = intersectPoint(nodes(end + i - 2, 1), nodes(end + i - 2, 2), nodes(j,1), nodes(j,2), map(k, 1), map(k, 2), map(k, 3), map(k, 4));
            end
            if any(isect), d(j) = -1; continue; end
            d(j) = norm(nodes(j, :) - nodes(end + i - 2, :));
            isect = [];
        end
        ind(i) = find(d == min(d(d>0))); w(i) = d(ind(i));
    end
    G = addedge(G, ind, [size(nodes, 1) - 1 size(nodes, 1)], w);
    unvisitNodes = 1:size(nodes, 1); C = Inf*ones(size(nodes, 1), 1); C(end - 1) = 0; nodeID = size(nodes, 1) - 1; unvisitNodes(end - 1) = [];
    while ~isempty(unvisitNodes)
        newNode = [];
        for i = 1:length(nodeID)
            [eid, nid] = outedges(G, nodeID(i));
            C(nid(C(nodeID(i)) + G.Edges(eid, 2).(1) < C(nid))) = G.Edges(eid(C(nodeID(i)) + G.Edges(eid, 2).(1) < C(nid)), 2).(1) + C(nodeID(i));
            newNode = [newNode; nid];
        end
        nodeID = unvisitNodes(ismember(unvisitNodes, newNode))'; unvisitNodes = setdiff(unvisitNodes,nodeID);
    end
    nodeID = size(nodes, 1); path = nodeID;
    while nodeID ~= size(nodes, 1) - 1
        [~, nid] = outedges(G, nodeID); nid = setdiff(nid, path);
        [~, minId] = min(C(nid));
        nodeID = nid(minId); path = [path; nodeID];
    end
end