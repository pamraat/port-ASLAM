function H = HJacBeacon(x, map)
    eps = [1e-3 1e-3 1e-3];
    for i=1:3
        a = zeros(3, 1); a(i)= eps(i);
        H(:, i) = (hBeacon(x + a, map) - hBeacon(x, map))/eps(i);
    end
end