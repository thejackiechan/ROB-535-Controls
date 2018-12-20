function [i,dist] = closestCenterlinePoint(x, cline)
    % Returns the index (i) of the centerline point closest to the
    % current position (x) and its distance (dist)
    
    D = sqrt(sum((x - cline) .^ 2))
    
    [dist, i] = min(D);
    
end