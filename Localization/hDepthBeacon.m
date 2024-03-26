function z = hDepthBeacon(x, id, map, mapBeacon)
    sensor_pos = [0 0.08]; angles = linspace(27, -27, 9)'*pi/180;
    z(:, 1) = depthPredict(x, map, sensor_pos, angles);
    z = [z; hBeacon(x, id, map(5:end, :), mapBeacon)];
end