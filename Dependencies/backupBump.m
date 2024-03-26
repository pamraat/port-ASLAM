function[fwdVelnew, angVelnew, timrev, timturn] = ...
    backupBump(dataStore, fwdVel, angVel, timrev, timturn)

if(dataStore.bump(end,2) == 1 || dataStore.bump(end,3) == 1 || dataStore.bump(end,7) == 1)
        timrev = dataStore.bump(end,1);
end

if(timrev ~= -1)
    if(dataStore.bump(end,1) < timrev + 0.25/abs(fwdVel))
        fwdVelnew = -fwdVel;
        angVelnew = 0;
    else
        timrev = -1;
        fwdVelnew = 0;
        angVelnew = -1;
        timturn = dataStore.bump(end,1);
    end
elseif(timturn ~= -1)
    if(dataStore.bump(end,1) < timturn + pi/3)
        fwdVelnew = 0;
        angVelnew = -0.5;
    else
        fwdVelnew = fwdVel;
        angVelnew = angVel;
        timturn = -0.5;
    end
else
    fwdVelnew = fwdVel;
    angVelnew = angVel;
end