function [map, lim] = modLine(map, t, robotRad, f)
    [minX, maxX] = bounds([map(1:4, 1); map(1:4, 3)]); [minY, maxY] = bounds([map(1:4, 2); map(1:4, 4)]);
    lim = [minX + robotRad, minY + robotRad, maxX - robotRad, maxY - robotRad]; map = map(5:end, :);
    perp = [map(:, 4) - map(:, 2), map(:, 3) - map(:, 1)]; perp = perp./sum((perp.^2), 2).^0.5;
    robotRad = f*robotRad;
    map = [map(:, 1:2) + (robotRad + t/2)*perp, map(:, 3:4) + (robotRad + t/2)*perp, map(:, 3:4) - (robotRad + t/2)*perp, map(:, 1:2) - (robotRad + t/2)*perp];
    map = map';
end