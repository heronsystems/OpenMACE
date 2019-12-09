% This function update the plot at F3 using location and attitude obtained 
% by the callback functions + global variable approach

% Sheng Cheng, Dec. 2019

function updatePlot(ROS_MACE)

    global tStart;
    global agentYawAngle;
    global agentPosition;
    
    plotMemory = 5; % number of markers of one trajectory left in the plot
    yawIndicatorLength = 2;
    
    persistent positionPlotHandle;
    if isempty(positionPlotHandle)
        positionPlotHandle = nan(ROS_MACE.N,plotMemory); 
        % each row contains the handles to the position plot of an agent 
        % whereas each column stands for the plot at each moment. The 1st,
        % 2nd, and 3rd column represents the current plot, previous plot,
        % and the previously previous plot, respectively.
    end
    
    persistent yawPlotHandle;
    if isempty(yawPlotHandle)
        yawPlotHandle = nan(ROS_MACE.N,1);
        % each row contains the handle to the bar for yaw indicator
    end
    
    
    colors=['rbkmgcyrbkmgcyrbkmgcyrbkmgcyrbkmgcyrbkmgcyrbkmgcy'];
    time = toc(tStart);
    % ------ plot altitude ------
    for jj = 1:ROS_MACE.N
        plot(ROS_MACE.altitude,time,agentPosition(jj,3), 'o','MarkerEdgeColor', colors(jj));
        hold on;
        if ( time > 30 )
            xlim(ROS_MACE.altitude,[time-30 time]);
        end
    end
    drawnow;
    
    % ------ plot position ------
    % update the visibility of the outdated plots
    if all(~sum(isnan(positionPlotHandle)))
        % when all elements of positionPlotHandle are filled, set the last
        % column to be invisible.
        for jj = 1:ROS_MACE.N
            set(positionPlotHandle(jj,plotMemory),'Visible','off');
        end
    end
    % then remove the last column
    positionPlotHandle(:,plotMemory) = [];
    
    % insert placeholder for the plots of current position
    positionPlotHandle = [nan(ROS_MACE.N,1) positionPlotHandle];
    
    % add plot of current position
    for jj = 1:ROS_MACE.N
        posHandle = scatter(ROS_MACE.taskAndLocation,agentPosition(jj,1),agentPosition(jj,2)', 'o','MarkerEdgeColor', colors(jj));
        hold on;
        positionPlotHandle(jj,1) = posHandle;
    end
    
    if all(~sum(isnan(positionPlotHandle)))       
        % adjust the transparency of earlier plots
        for kk = 2:plotMemory
            for jj = 1:ROS_MACE.N
                set(positionPlotHandle(jj,kk),'MarkerEdgeAlpha',1-1/(plotMemory+1)*kk);
            end
        end
    end
    
    % ------ plot orientation ------
    for jj = 1:ROS_MACE.N
        if ~isnan(yawPlotHandle(jj))
            set(yawPlotHandle(jj),'Visible','off');
        end
        yawIndicator = [agentPosition(jj,1:2);agentPosition(jj,1:2)+yawIndicatorLength*[cos(agentYawAngle(jj)) sin(agentYawAngle(jj))]];
        yawPlotHandle(jj) =  plot(ROS_MACE.taskAndLocation,yawIndicator(:,1),yawIndicator(:,2),'Color',colors(jj));
    end
    drawnow;
    
end