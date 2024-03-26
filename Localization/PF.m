function Xnew = PF(X, u, z, R, Q, g, h, map, mapBeacon)
    try
        [tempMap, lim] = modLine(map, 0.1, 0.01, 0); modMap = point2Line(tempMap, lim); XPred = []; nParticles = size(X, 2); z(z == 0) = 0.16;
        if z(end) > 0
            beacon = mapBeacon(mapBeacon(:, 3) == z(end), 1:2); r = norm(z(end - 2:end - 1)); [~, Xmaxind] = max(X(end, :));
            while size(XPred, 2) < floor(nParticles*0.7)
                theta = 2*pi*rand(1, 1); rn = r + mvnrnd(0, Q(end-1, end-1)); x = beacon(1) + rn*cos(theta); y = beacon(2) + rn*sin(theta);
                if edgeFree([x y], beacon, map(5:end, :)) && pointFree([x y], modMap)
                    pose3 = g(X(1:end - 1, Xmaxind), u) + mvnrnd(zeros(3, 1), R)';
                    XPred = [XPred [x; y; pose3(3)]];
                    XPred(1:2, end) = robot2global(XPred(:, end),[0 -0.08])';
                end
            end
            for i = size(XPred, 2) + 1:nParticles
                XPred(:, i) = g(X(1:end - 1, i), u) + mvnrnd(zeros(3, 1), R)';
            end
        else
            for i = 1:nParticles
                XPred(:, i) = g(X(1:end - 1, i), u) + mvnrnd(zeros(3, 1), R)';
            end
        end
        w = zeros(1, nParticles);
        for i = 1:nParticles
            zPred(:, i) = h(XPred(:, i), z(end), map, mapBeacon);
            if(any(isnan(zPred(:, i)))), zPred(1:end-2, i) = 1000; end
            w(1, i) = mvnpdf(zPred(:, i), z(1:end - 1), Q);
        end
        if all(w == 0, 'all'), Xnew = [XPred; 1/nParticles*ones(1, nParticles)]; return; end
        isFree = pointFree([XPred(1, :)' XPred(2, :)'], modMap); w(1, ~isFree) = 0;
        w = w/sum(w); sampInd = randsample(nParticles, nParticles, true, w);
        Xnew = [XPred(:, sampInd); w(sampInd)];
    catch
        Xnew = X;
    end
end