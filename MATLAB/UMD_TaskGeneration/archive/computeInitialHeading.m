function [hv] = computeInitialHeading(xv,yv,prevPt,xpoly,ypoly,angOffset)
N = length(xv);
for i = 1:1:N
    [status] = isPointAlongCorner(xv(i),yv(i),xpoly,ypoly);
    if ( status ) % true
        [h1, h2] = headingRangeAtCorner(xpoly, ypoly, prevPt(i));
    else
        [h1, h2] = headingRangeAlongSeg(prevPt(i), xpoly, ypoly);
    end
    [h1,h2] = shrinkInterval(h1,h2,angOffset);
    hv(i) = randomHeadingCW(h1,h2);
end

end