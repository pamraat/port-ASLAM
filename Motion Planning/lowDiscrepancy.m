function nodes = lowDiscrepancy(n, map, lim)
    p = haltonset(2); nodes = []; i = 2;
    while size(nodes, 1) < n
        point = [(lim(3) - lim(1))*p(i, 1) + lim(1), (lim(4) - lim(2))*p(i, 2) + lim(2)];
        if pointFree(point, map), nodes = [nodes; point]; end
        i = i + 1;
    end
end