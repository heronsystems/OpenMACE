function trueWorld = loadEnvironment(trueWorld, targetModel)
if ( exist([trueWorld.folder trueWorld.fileName '_' trueWorld.type '_full.mat'],'file') == 0 )
    disp('Creating/parsing new trueWorld node locations...');
    LatRef = [];
    LongRef = [];
    switch trueWorld.type
        case 'cityblocks'
            nodeXY = loadCityBlocksNodes(trueWorld.blockLength, trueWorld.numBlocks, trueWorld.binWidth);
            [ G_env, A_env, nodeX, nodeY ] = convertNodesXYtoGraph(nodeXY, trueWorld.borderOffset, trueWorld.binWidth );    
        case 'openStreetMap'
            [ nodeXY, LatRef, LongRef, G_env, A_env] = loadOpenStreetMapNodesFlex(trueWorld.fileName, trueWorld.refX, trueWorld.refY, trueWorld.boxlength, trueWorld.boxwidth, trueWorld.angle, trueWorld.binWidth, trueWorld.removeList);            
            nodeX = nodeXY(:,1);
            nodeY = nodeXY(:,2);
        case 'cityblocksAtF3'
            [ nodeXY ] = loadCityBlocksNodes_atF3(trueWorld.blockLength, trueWorld.numBlocks, trueWorld.binWidth, trueWorld.f3Workspace);
            [ G_env, A_env, nodeX, nodeY ] = convertNodesXYtoGraph(nodeXY, trueWorld.borderOffset, trueWorld.binWidth );    
        case 'osmAtF3'
            [ nodeXY ] = loadOpenStreetMapNodes_atF3(trueWorld.fileName, trueWorld.refX, trueWorld.refY, trueWorld.boxlength, ...
                trueWorld.angle, trueWorld.buffer, trueWorld.binWidth, trueWorld.f3Workspace);
            [ G_env, A_env, nodeX, nodeY ] = convertNodesXYtoGraph(nodeXY, trueWorld.borderOffset, trueWorld.binWidth );   
        case 'randomRoadsAtF3'
            [nodeXY] = loadRandomRoads_atF3(trueWorld.nodeFile, trueWorld.edgeFile, trueWorld.binWidth, trueWorld.f3Workspace);
            [ G_env, A_env, nodeX, nodeY ] = convertNodesXYtoGraph(nodeXY, trueWorld.borderOffset, trueWorld.binWidth );
    
    end
    disp('Computing Additional Properties for trueWorld...');
    
    [xpoly,ypoly] = buildRectangularBoundary(nodeX, nodeY, trueWorld.borderOffset);
    trueWorld.nodeXY = nodeXY;
    trueWorld.G_env = G_env;
    trueWorld.A_env = A_env;
    trueWorld.nodeX = nodeX;
    trueWorld.nodeY = nodeY;
    trueWorld.xpoly = xpoly;
    trueWorld.ypoly = ypoly;
    if ( ~isempty(LatRef) )
        trueWorld.LatRef = LatRef;
        trueWorld.LongReg = LongRef;
    end
    trueWorld.minX = min(trueWorld.xpoly);
    trueWorld.maxX = max(trueWorld.xpoly);
    trueWorld.minY = min(trueWorld.ypoly);
    trueWorld.maxY = max(trueWorld.ypoly);
    trueWorld.numNodes = numnodes(trueWorld.G_env);
    
    % the whole environment is discretized into this many bins:
    trueWorld.numBinsX = floor( (max(trueWorld.xpoly) - min(trueWorld.xpoly))/trueWorld.binWidth ) + 1; 
    trueWorld.numBinsY = floor( (max(trueWorld.ypoly) - min(trueWorld.ypoly))/trueWorld.binWidth ) + 1; 
    
    % (xcp, ycp) compute the control pts corresponding to the center of each bin
    % numNodesMat gives number of nodes in each bin
    % bin2NodeID returns nodeIndex given bin coordinates
    disp('Creating Search Grid...');
    [trueWorld.xcp,trueWorld.ycp,trueWorld.numNodesMat,trueWorld.bin2NodeID] = createSearchGrid(trueWorld.xpoly, ...
        trueWorld.ypoly, trueWorld.nodeX, trueWorld.nodeY, trueWorld.numBinsX, trueWorld.numBinsY);
    [trueWorld.xx,trueWorld.yy] = meshgrid(trueWorld.xcp, trueWorld.ycp);
    trueWorld.binWidth = trueWorld.ycp(2) - trueWorld.ycp(1);
    
    disp('Building Target State Space...');
    % motion model parameters
    [ trueWorld.G_tss, trueWorld.Mp, trueWorld.Mc  ] = environmentToStateSpace( trueWorld.G_env );
    trueWorld.Ns = numnodes(trueWorld.G_tss);
    trueWorld.A = adjacency(trueWorld.G_tss);
    % compute state transition matrix given motion model parameters and state
    % space
    disp('Created state transition matrix...')
    [ trueWorld.Q ] = stateTransitionMatrix ( trueWorld.G_tss, targetModel.probStopping, targetModel.m , targetModel.d );
    

    save([trueWorld.folder trueWorld.fileName '_' trueWorld.type '_full.mat'],'trueWorld','-v7.3');
    disp(['Saved new "trueWorld" (full model) to:' trueWorld.folder trueWorld.fileName '_' trueWorld.type '_full.mat']);
else
    load([trueWorld.folder trueWorld.fileName '_' trueWorld.type '_full.mat'],'trueWorld')
    disp(['Loaded exsting "trueWorld" (full model) from:' trueWorld.folder trueWorld.fileName '_' trueWorld.type '_full.mat']);
end


end
