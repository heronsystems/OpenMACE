function nodeXYmod = clipMap( nodeXY, polyXY )
% clip
nodeXYmod = [];
k = 1;
for i = 1:1:length(nodeXY)
    if (    nodeXY(i,1) >= clipXmin && nodeXY(i,1) <= clipXmax ...
         && nodeXY(i,2) >= clipYmin && nodeXY(i,2) <= clipYmax )
        nodeXYmod(k,1) = nodeXY(i,1);
        nodeXYmod(k,2) = nodeXY(i,2);
        k = k + 1;
    end
end
% scale and shift
nodeXYmod(:,1) = scale*nodeXYmod(:,1) + shiftX;
nodeXYmod(:,2) = scale*nodeXYmod(:,2) + shiftY;


end
