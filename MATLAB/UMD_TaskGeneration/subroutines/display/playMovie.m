function playMovie(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel,spArray,spTypes)

% initialize each subplot
for i = 1:1:length(spArray)
    plotHandles{i}.subplotHandle = spArray(i);
    % initialize states
    k = 1;
    swarmState = swarmStateHist{k};
    targetState = targetStateHist{k};
    swarmWorld = swarmWorldHist{k};
    plotHandles{i} = eval([ 'init' spTypes{i} 'Movie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles{i});']);    
end


drawnow;
pause;
% loop each movie
saveFramesFlag = 0;

for k = 1:1:length(swarmStateHist)
    tic
    swarmState = swarmStateHist{k};
    targetState = targetStateHist{k};
    swarmWorld = swarmWorldHist{k};
    for i = 1:1:length(spArray)
        plotHandles{i} = eval([ 'update' spTypes{i} 'Movie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles{i});']);
    end
    drawnow;
%     if ( saveFramesFlag ) 
%         frameFileName = sprintf('./frames/frame_%06d',k);
%         print(gcf,frameFileName,'-dpng')
%     end
    toc
   pause(0.1);
   %pause;
    fprintf('Frame %i of %i \n',k, length(swarmStateHist)-1);
end
    
end

