function nodes = lowDispersion(n, map)
    [minX, maxX] = bounds([map(:, 1); map(:, 3)]); [minY, maxY] = bounds([map(:, 2); map(:, 4)]);
    lim = [minX minY maxX maxY]; m = ceil(sqrt(n)); nodes = [];
    while size(nodes, 1) < n
        nodes = [];
        x = linspace(lim(1), lim(3), m); y = linspace(lim(2), lim(4), m);
        xC = linspace(x(1), x(end - 1), m) + (x(end) - x(1))/(2*m); yC = linspace(y(1), y(end - 1), m) + (y(end) - y(1))/(2*m);
        XGrid = repmat(xC, 1, length(yC))'; YGrid = repelem(yC, 1, length(xC))'; ind = pointFree([XGrid YGrid], map);
        XGrid = XGrid(ind); YGrid = YGrid(ind);
        while ~isempty(XGrid) && size(nodes, 1) < n
            for i = 1:m
                ind = find(XGrid == xC(i)); if ~isempty(ind), ind = randsample(ind, 1); end
                nodes = [nodes; [XGrid(ind) YGrid(ind)]]; XGrid(ind) = []; YGrid(ind) = [];
                if isempty(XGrid) || size(nodes, 1) == n, break; end
            end
        end
        m = m + 1; 
    end
end