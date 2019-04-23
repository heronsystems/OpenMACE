function [wpts, swarmWorld] = taskGeneration(swarmWorld, swarmModel, trueWorld)
switch swarmModel.taskGeneration
    case 'lawnmower'
        wpts = [];        
    case 'randomWpts'
        wpts = []; 
    case 'randomBoundaryWpts'
        wpts = [];
    case 'mutualInfoWpts' 
        swarmWorld.samplingPriority = swarmWorld.mutualInfoSurface;
        temp = tic;
        if ( ~isfield(swarmWorld,'cellCenterOfMass') )        
        % compute voronoi partition for the first time
        disp('running for first time / VP');
        [swarmWorld.voronoiVertices, swarmWorld.voronoiCells, swarmWorld.cellMass, swarmWorld.cellCenterOfMass] = ...
            approxEqualMassVornoiPartition(trueWorld.xx, trueWorld.yy, ...
                swarmWorld.samplingPriority, ...
                swarmModel.numTasks, ...
                swarmModel.stepSizeGain, ...
                swarmModel.percentTol, ...
                swarmModel.maxIters);
        else
        disp('running recursive / VP');            
            % compute voronoi partition iteration (extra argument)
            [swarmWorld.voronoiVertices, swarmWorld.voronoiCells, swarmWorld.cellMass, swarmWorld.cellCenterOfMass] = ...
                approxEqualMassVornoiPartition(trueWorld.xx, trueWorld.yy, ...
                swarmWorld.samplingPriority , ...                 %swarmWorld.mutualInfoSurface, ...
                swarmModel.numTasks, ...
                swarmModel.stepSizeGain, ...
                swarmModel.percentTol, ...
                swarmModel.maxIters, swarmWorld.cellCenterOfMass);               
        end     
        timeStamp = toc(temp);
        fprintf('approxEqualMaxxVoronoiPartition took %3.3f s\n',timeStamp);
        
        wpts = swarmWorld.cellCenterOfMass;
    case 'likelihoodWpts' 
        swarmWorld.samplingPriority = samplingPriority( swarmWorld );
        if ( ~isfield(swarmWorld,'cellCenterOfMass') )        
        % compute voronoi partition for the first time
        [swarmWorld.voronoiVertices, swarmWorld.voronoiCells, swarmWorld.cellMass, swarmWorld.cellCenterOfMass] = ...
            approxEqualMassVornoiPartition(trueWorld.xx, trueWorld.yy, ...
                swarmWorld.samplingPriority, ...
                swarmModel.numTasks, ...
                swarmModel.stepSizeGain, ...
                swarmModel.percentTol, ...
                swarmModel.maxIters);    
        else
        % compute voronoi partition iteration (extra argument)
        [swarmWorld.voronoiVertices, swarmWorld.voronoiCells, swarmWorld.cellMass, swarmWorld.cellCenterOfMass] = ...
            approxEqualMassVornoiPartition(trueWorld.xx, trueWorld.yy, ...
                swarmWorld.samplingPriority , ...                 %swarmWorld.mutualInfoSurface, ...
                swarmModel.numTasks, ...
                swarmModel.stepSizeGain, ...
                swarmModel.percentTol, ...
                swarmModel.maxIters, swarmWorld.cellCenterOfMass);               
        end                        
        wpts = swarmWorld.cellCenterOfMass;       
    case 'frontierWpts'       
        if strcmp(swarmModel.mapping.method,'frontierOnly')
            % frontier only
            wpts = [trueWorld.nodeX(swarmWorld.frontierIndex) trueWorld.nodeY(swarmWorld.frontierIndex)];
        else
            % frontier + blob
            wpts = [trueWorld.nodeX(swarmWorld.frontierIndex) trueWorld.nodeY(swarmWorld.frontierIndex)];
            % filter out the blob whose area is greater than 4*(sensing
            % area)
            interestBlob = bwareafilt((swarmWorld.cellStateMat==2),[round(swarmModel.mapping.minBlobArea) inf]);
            % label blobs
            labelBlob = bwlabel(interestBlob);
            % count the totoal number of blobs
            NBlob = max(max(labelBlob));
            % partition each blob into subblobs whose size approximately equals 4*(sensing area)
            for k = 1:NBlob
                % extract location of pixels in the blob
                [rowBlob,colBlob] = find(labelBlob == k);
                % compute the total area of the blob
                blobArea = regionprops((labelBlob == k),'Area');
                blobArea = [blobArea.Area(:,1)];
                % extract location of the centroids of each subblob
                [idx,centroidSubblob] = kmeans([rowBlob,colBlob],round(blobArea/swarmModel.mapping.minBlobArea));
                        
                if swarmModel.mapping.maxMajorMinorAxisRatio < inf
                    % filter out the subblob that is too thin
                    for j = 1:max(idx)
                        % recover the pixels in a subblob
                        rowSubBlob = rowBlob((idx==j));
                        colSubBlob = colBlob((idx==j)); 
                                                       
                        % translate the subblob (this step avoids directly
                        % using the original binary image which for sure has a lot of zeros) 
                        rowSubBlobMin = min(rowSubBlob);
                        rowSubBlobMax = max(rowSubBlob);
                        colSubBlobMin = min(colSubBlob);
                        colSubBlobMax = max(colSubBlob);  

                        % generate the subblob
                        subBlob = zeros(rowSubBlobMax-rowSubBlobMin+1,colSubBlobMax-colSubBlobMin+1);
                        subBlob(rowSubBlob-rowSubBlobMin+1,colSubBlob-colSubBlobMin+1) = 1;    

                        % compute the length (in pixels) of the major axis and
                        % the minor axis of the ellipse that has the same
                        % normalized second central moments as the subblob
                        majorAxisLength = regionprops(subBlob,'MajorAxisLength');
                        majorAxisLength = majorAxisLength(1).MajorAxisLength;
                        minorAxisLength = regionprops(subBlob,'MinorAxisLength');
                        minorAxisLength = minorAxisLength(1).MinorAxisLength;                                
                        % eliminate the thin subblobs
                        if majorAxisLength/minorAxisLength < swarmModel.mapping.maxMajorMinorAxisRatio
                            % add these centroids to waypoints (as future tasks)
                            wpts = [wpts; [centroidSubblob(j,2)+trueWorld.minX centroidSubblob(j,1)+trueWorld.minY]];
                        end
                    end
                else
                    % do not apply filter
                    % add these centroids to waypoints (as future tasks)
                    wpts = [wpts; [centroidSubblob(:,2)+trueWorld.minX centroidSubblob(:,1)+trueWorld.minY]];
                end
            end
            % record locations of these centroids
            swarmWorld.subblobCentroid = wpts(length(swarmWorld.frontierIndex)+1:end,:);
        end
    end
end
