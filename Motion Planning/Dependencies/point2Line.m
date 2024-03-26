function [mapLine, id] = point2Line(map, lim)
    map = [map; zeros(4, size(map, 2))];
    for i = 1:size(map, 2)
        map(find(map(:, i) == 0, 2), i) = [map(1, i); map(2, i)];
    end
    for i = 4:2:size(map, 2)*size(map, 1)
        id(i/2 - 1, 1) = floor(i/size(map, 1)) + 1;
        mapLine(i/2 - 1, :) = map(i-3:i);
    end
    id = id(all(mapLine, 2)); mapLine = mapLine(all(mapLine, 2), :);
    mapLine = [[lim(1) lim(2) lim(3) lim(2); lim(3) lim(2) lim(3) lim(4); lim(3) lim(4) lim(1) lim(4); lim(1) lim(4) lim(1) lim(2)]; mapLine];
end