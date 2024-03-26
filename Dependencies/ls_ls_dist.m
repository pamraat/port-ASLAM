function dist = ls_ls_dist(x1, y1, x2, y2, x3, y3, x4, y4)
    [intersects, ~, ~] = intersectPoint(x1, y1, x2, y2, x3, y3, x4, y4);
    if intersects == true
        dist = 0;
    else
        dist_1 = ls_point_dist(x3, y3, x4, y4, x1, y1);
        dist_2 = ls_point_dist(x3, y3, x4, y4, x2, y2);
        dist_3 = ls_point_dist(x1, y1, x2, y2, x3, y3);
        dist_4 = ls_point_dist(x1, y1, x2, y2, x4, y4);
        dist = min([dist_1 dist_2 dist_3 dist_4]);
    end
end