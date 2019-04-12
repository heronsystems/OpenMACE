function [hitStatus, bounceDir, minDist] = hitsWall(x,xpoly,ypoly,tol,bounceType)
[minDist, segPtA, segPtB ] = distToPolygon(x(1), x(2), xpoly, ypoly);
hitStatus = 0;
bounceDir = [];
if ( minDist <= tol )
    hitStatus = 1;
    % assume pt.A and pt. B define boundary counter-clockwise 
    hNormal = normalToLine( xpoly(segPtA), ypoly(segPtA), xpoly(segPtB), ypoly(segPtB), 'left' );
    if ( strcmp(bounceType,'normal') )
        bounceDir = hNormal;
    elseif ( strcmp(bounceType,'snellsLaw') )
        incidenceAngle = signedAngularDist( x(3) + pi , hNormal );
        bounceDir = mod ( x(3) + pi + 2*incidenceAngle , 2*pi );
    end
end

end