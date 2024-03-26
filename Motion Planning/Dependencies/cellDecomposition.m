function [cellDecomp, mapLine] = cellDecomposition(map, lim)
    [mapLine, id] = point2Line(map, lim);
    x = mapLine(5:end, 1); y = mapLine(5:end, 2);
    cellDecomp = [];
    for i = 1:length(x)
        d1 = depthPredict([x(i), y(i), pi/2], mapLine, [0 0], 0);
        d2 = depthPredict([x(i), y(i), -pi/2], mapLine, [0 0], 0);
        [in1, ~] = inpolygon(x(i), y(i) + 0.01, x(id == id(i)), y(id == id(i)));
        [in2, ~] = inpolygon(x(i), y(i) - 0.01, x(id == id(i)), y(id == id(i)));
        if(~in1), cellDecomp = [cellDecomp; [x(i) y(i) + d1 x(i) y(i)]]; end
        if(~in2), cellDecomp = [cellDecomp; [x(i) y(i) x(i) y(i) - d2]]; end
    end
    [~, iCell] = sort(cellDecomp(:, 1)); cellDecomp = cellDecomp(iCell, :);
end