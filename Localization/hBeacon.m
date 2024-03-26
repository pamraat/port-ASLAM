function z = hBeacon(x, id, map, mapBeacon)
    if (id<0), z = [-1; -1]; return; end
    sensor_pos = [0, 0.08];
    z = global2robot(x, mapBeacon(mapBeacon(:, 3) == id, 1:2));
    z = global2robot([sensor_pos 0], z)';
    in = inpolygon(z(1, 1), z(2, 1), [0; 1000*cos(27/180*pi); 1000*cos(27/180*pi)], [0; -1000*sin(27/180*pi); 1000*sin(27/180*pi)]);
    isFree = edgeFree(mapBeacon(mapBeacon(:, 3) == id, 1:2), x(1:2)', map);
    if (in && isFree), return; end
    z = [-1; -1];
end