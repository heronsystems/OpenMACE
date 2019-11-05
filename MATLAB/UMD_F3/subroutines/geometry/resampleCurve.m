function [x,y] = resampleCurve(xc,yc,dx)
    s = arclengthCurve(xc,yc,0);
    N = max(2, floor(s/dx));
    [x, y, ~] = distributeUniformlyAlongCurve(N,xc,yc);
end