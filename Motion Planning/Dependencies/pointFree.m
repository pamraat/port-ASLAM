function free = pointFree(points, map)
    inMap = map(5:end, :); i = 1; k(1) = 0;
    while k(i) < size(inMap, 1)
        [~, k(i + 1)] = ismember(inMap(k(i) + 1, 1:2), inMap(:, 3:4), 'rows');
        i = i + 1;
    end
    for i = 2:length(k)
        isFree(:, i - 1) = inpolygon(points(:, 1), points(:, 2), inMap(k(i - 1) + 1:k(i), 1), inMap(k(i - 1) + 1:k(i), 2));
    end
    free = logical(~any(isFree, 2))';
    inBounds = inpolygon(points(:, 1), points(:, 2), map(1:4, 1), map(1:4, 2))';
    free = free & inBounds;
end