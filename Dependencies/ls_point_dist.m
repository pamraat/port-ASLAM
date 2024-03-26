function dist = ls_point_dist(x1, y1, x2, y2, x3, y3)
    u = ((x3-x1)*(x2-x1)+(y3-y1)*(y2-y1))/norm([x2, y2]-[x1, y1])^2;
    if u > 1
        u = 1;
    elseif u < 0
        u = 0;
    end
    x = x1 + u*(x2-x1);
    y = y1 + u*(y2-y1);
    dist = norm([x, y] - [x3, y3]);
end