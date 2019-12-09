function taskList = updateTaskList(exploredGraph,nodeX,nodeY)
%
% Description: generate a vector that contains the XY
% coordinates of the tasks (frontier nodes) whose number is Ntask
%
% Input:
%   exploredGraph : a graph object that records the current explored graph
%   nodeX: X coordinates of all nodes
%   nodeY: TYcoordinates of all nodes
% Output:
%   taskList: a (Ntask x 2) vector containing the XY coordinates of 
%   the tasks (frontier nodes) whose number is Ntask
%
%
% 
% Sheng Cheng, 2018

   
    
% The variable indexForFrontierNodes contains the id of the frontier nodes
% which connects to the unexplored nodes
[indexForFrontierNodes,~] = find(~cellfun(@isempty,strfind(table2array(exploredGraph.Nodes),'U')) + ...
                                 ~cellfun(@isempty,strfind(table2array(exploredGraph.Nodes),'D')) + ...
                                 ~cellfun(@isempty,strfind(table2array(exploredGraph.Nodes),'L')) + ...
                                 ~cellfun(@isempty,strfind(table2array(exploredGraph.Nodes),'R')));
       
% parse the frontier edges (i.e., remove UDLR)
temp = table2array(exploredGraph.Nodes);
frontierNodes = temp(indexForFrontierNodes);
frontierNodes = regexp(frontierNodes,'\d*','Match');
    
% initialize the taskList
taskList = zeros(length(frontierNodes),2);
    
% assign waypoints of the frontier nodes to the taskList (XY coordinates)
if ~isempty(frontierNodes)
    for k = 1:length(frontierNodes)
        temp = str2num(cell2mat(frontierNodes{k,1}));
        taskList(k,:) = [nodeX(temp),nodeY(temp)];
    end
end
    
