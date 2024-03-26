% clf
% clear
% clc
% hold on
% 
% % line = [-1 1 1 1]
% % plot([line(1) line(3)], [line(2) line(4)])
% % xmin = line(1)
% % xmax = line(3)
% % ymin = line(2)
% % ymax = line(4)
% 
% %% Plot
%load 'loopMap.mat'
% load practiceMap_4credits.mat
% %map = loopMap;
% map = map;
% %map = find_optional_walls(map, optWalls, 50, 50)
% assc = associate_cells_optwalls(map, optWalls, 50, 50);
% display(assc(1,:,:))
function optwall_cell_matrix = associate_cells_optwalls(map, optWalls, m, n)
    max_a = max([map(:,1);map(:,2);map(:,3);map(:,4)]);
    min_a = min([map(:,1);map(:,2);map(:,3);map(:,4)]);
%     x = linspace(min([map(:,1);map(:,3)]), max([map(:,1);map(:,3)]), n + 1);
%     y = linspace(min([map(:,2);map(:,4)]), max([map(:,2);map(:,4)]), m + 1);
    x = linspace(min_a, max_a, n + 1);
    y = linspace(min_a, max_a, m + 1);
    xC = linspace(x(1), x(end - 1), n) + (x(end) - x(1))/(2*n);
    yC = linspace(y(1), y(end - 1), m) + (y(end) - y(1))/(2*m);
    threshold = (xC(2)-xC(1))/2
    [X, Y] = meshgrid(x, y);
    optwall_cell_matrix = zeros(size(optWalls,1),size(X,1),size(X,2));
    [XC, YC] = meshgrid(xC, yC);
    C = [0.6 0.6 0.6];
    plot(X, Y, 'Color', C)
    plot(X', Y', 'Color', C)
    for i = 1:size(optWalls, 1)
        plot([optWalls(i, 1) optWalls(i, 3)], [optWalls(i, 2) optWalls(i, 4)], LineWidth=1, Color='black');
    end
    %scatter(XC, YC)
    for i = 1:1:size(xC,2)
        for j = 1:1:size(yC,2)
            for k = 1:1:size(optWalls,1)
%                 display("point")
%                 xC(i)
%                 yC(j)
%                 ls_point_dist(optWalls(k,1), optWalls(k,2), optWalls(k,3), optWalls(k,4), xC(i), yC(j))
%                 display("endpt")
                if ls_point_dist(optWalls(k,1), optWalls(k,2), optWalls(k,3), optWalls(k,4), xC(i), yC(j)) < threshold
                    scatter(xC(i), yC(j))
                    optwall_cell_matrix(k,j,i) = 1;
%                     display(k)
%                     display(j)
%                     display(i)
                end
            end
        end
    end
end