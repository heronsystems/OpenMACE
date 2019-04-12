function nodeXY = loadCityBlocksNodes(blockLength, numBlocks, L)
    base = [L/2:L:blockLength*numBlocks+L/2]';
    nodeXY = [];
    % build E/W streets
    for i = 1:1:numBlocks+1
        nodeXY = [nodeXY; base, (i-1)*blockLength.*ones(size(base))+L/2 ];
    end
    % build N/S streets
    for i = 1:1:numBlocks+1
        nodeXY = [nodeXY; (i-1)*blockLength.*ones(size(base))+L/2, base ];
    end
    % remove duplicate intersections
    nodeXY = unique(nodeXY,'rows');
    plot(nodeXY(:,1), nodeXY(:,2),'o')
end