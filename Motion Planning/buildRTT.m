function wayPoints = buildRTT(init, goal, step, robotRad, map, lim)
    for i = 1:size(map, 2)
        temp = map(any(map(:, i), 2), i); temp = reshape(temp, 2, length(temp)/2)';
        side = [temp(2:end, :); temp(1, :)] - temp; side = side./(sum(side.^2, 2)).^0.5;
        bi = (side - [side(2:end, :); side(1, :)]);
        in = inpolygon(temp(:, 1) + 0.01*bi(:, 1), temp(:, 2) + 0.01*bi(:, 2), temp(:, 1), temp(:, 2));
        bi = (1 - 2*in).*bi./(sum(bi.^2, 2)).^0.5; bi = reshape(bi', 2*size(bi, 1), 1);
        map(1:length(bi), i) = map(1:length(bi), i) + robotRad*[bi(end-1:end); bi(1:end - 2)];
    end
    wayPoints = buildRTTpoint(init, goal, step, map, lim);
end