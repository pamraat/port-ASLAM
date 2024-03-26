function nodes = unifSample(n, map)
    [minX, maxX] = bounds([map(:, 1); map(:, 3)]); [minY, maxY] = bounds([map(:, 2); map(:, 4)]);
    lim = [minX minY maxX maxY]; nodes = [];
    while size(nodes, 1) < n 
        isFree = 0;
        while ~isFree
            q_s = [(lim(3) - lim(1))*rand(1) + lim(1), (lim(4) - lim(2))*rand(1) + lim(2)];
            isFree = pointFree(q_s, map);
        end
        nodes = [nodes; q_s];
    end
end