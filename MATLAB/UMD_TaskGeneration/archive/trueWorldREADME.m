function trueWorldREADME()

% Conventions 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% X points East
% Y points North
% Angles are measured CCW from X axis (right-hand rule)
% Angles are all in radians
% 'Width' of map refers to length in X-direction
% 'Height' of map refers to length in Y-direction

% trueWorld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This variable stores the structure of the environment:

% type : specifies how nodesXY locations were defined
% borderOffset : distanced used when adding padding around nodes to define
%                search grid
% maxDistConnectivity : distance used to define edge between two nodes
% folder : location where .mat file with parsed map is stored (for faster
%          loading)
% filename : name of raw file parsed to obtain nodesXY
% f3Workspace : determines if entire arena is used or just one third
% (refX, refY) : used when parsing open street map data to shift origin of
%   bounding box
% angle : angle of bounding box
% boxlenght : length of bounding box
% buffer : buffer inside bounding box used to omit defining nodes
% nodesXY are coordiantes of occupancy graph nodes
% (nodeX, nodeY) are coordiantes of occupancy graph nodes
% (xpoly,ypoly) give coordinates of bounding rectangle
% (minX, maxX, minY, maxY) are min/max values of (xpoly,ypoly)
% G_env : (true) Occupancy graph
% numNodes : number of nodes in G_env
% (numBinsX, numBinsY) is number of x and y control pts / cells
% (xcp, ycp) are control points at center of each cell
%   xcp : reads left to right (increasing)
%   ycp : reads bottom to top (increasing)
% numNodesMat has (numBinsY rows, and numBinsX cols) 
%   entry numNodesMat(by,bx) is 1 if xcp(bx) and ycp(by) is a node
% Note: Matrices represent "image" of pixel vals and increasing row number
% corresponds to increasing y value (thus needs to be flipped when plotted
% with imagesc)
% G_tss : (true) Target state space graph
% Ns : number of target states
% Mp : (Ns x numNodes) gives previous node index in G_env corresponding
% Mc : (Ns x numNodes) gives current node index in G_env corresponding
% Q : (Ns x Ns) state transition matrix 
% windowWidth : number of cells needed to consider Rsense range 


% e.g. 
% trueWorld = 
%   struct with fields:
% 
%             type: 'osmAtF3'
%     borderOffset: 0
%           folder: './data/'
%         binWidth: 1
%         fileName: 'RandallsIsland_Big.osm'
%      f3Workspace: 'full'
%             refX: -350
%             refY: 80
%            angle: -0.6458
%        boxlength: 500
%           buffer: 1
%           nodeXY: [392×2 double]
%            G_env: [1×1 graph]
%            nodeX: [392×1 double]
%            nodeY: [392×1 double]
%            xpoly: [5×1 double]
%            ypoly: [5×1 double]
%             minX: -57.5000
%             maxX: 26.5000
%             minY: -11.5000
%             maxY: 11.5000
%         numNodes: 392
%         numBinsX: 85
%         numBinsY: 24
%              xcp: [1×85 double]
%              ycp: [1×24 double]
%      numNodesMat: [24×85 double]
%       bin2NodeID: [24×85 double]
%               xx: [24×85 double]
%               yy: [24×85 double]
%            G_tss: [1×1 digraph]
%               Mp: [1198×392 double]
%               Mc: [1198×392 double]
%               Ns: 1198
%                Q: [1198×1198 double]
%      windowWidth: 7