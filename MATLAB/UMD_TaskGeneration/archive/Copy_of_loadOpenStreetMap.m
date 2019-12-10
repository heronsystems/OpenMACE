function [nodesXY, LatRef, LongRef] = loadOpenStreetMap(fileName, dx, nodeFactor, edgeFactor, dim )
disp('Parsing OpenStreetMap...')
% code adapted from openstreet map example
[parsed_osm, osm_xml] = parse_openstreetmap(fileName);
[bounds, node, way, ~] = assign_from_parsed(parsed_osm)
LatRef = mean(bounds(2,:));
LongRef = mean(bounds(1,:));
% define local origin in UTM coordinates
% Recall Lat indicates position north/south, Long is position east/west
% E.g., boston Lat 42.3601° N,  Long 71.0589° W
[xRef,yRef,utmzone] = deg2utm(LatRef, LongRef);
nodesXY = [];
for i=1:size(way.id, 2)
    [key, val] = get_way_tag_key(way.tag{1,i} );
    if ( strcmp(key, 'highway') )
        way_nd_ids = way.nd{1, i};
        num_nd = size(way_nd_ids, 2);
        nd_coor = zeros(2, num_nd);
        nd_ids = node.id;
        for j=1:num_nd
            cur_nd_id = way_nd_ids(1, j);
            if ~isempty(node.xy(:, cur_nd_id == nd_ids))
                nd_coor(:, j) = node.xy(:, cur_nd_id == nd_ids);
            end
        end
        % remove zeros
        nd_coor(any(nd_coor==0,2),:)=[];
        % nd_coor(2,:) are the Latitudes
        % nd_coor(1,:) are the Longitudes
        if ~isempty(nd_coor)
            [xc,yc,utmzone] = deg2utm(nd_coor(2,:),nd_coor(1,:));
            % resample the curve at internvals of length dx
            [x,y] = resampleCurve( xc, yc, dx);
            nodesXY = [ nodesXY; x y ];
        end
    end
end
% shift reference frame to the given origin, units are now meters
% easting (x) and northing (y)
nodesXY(:,1) = nodesXY(:,1) - xRef;
nodesXY(:,2) = nodesXY(:,2) - yRef;

% the default osm parsing above creates many duplicate nodes and nodes
% that are very close to each other. In this step we try to replace similar
% nodes with a single unique one. The distance for declaring two nodes as
% similar is Rthresh.
Rthresh = nodeFactor*dx;

% this while loop continues until 'n' the node index, reaches the end of 
% the node list
n = 1;
while ( n < length(nodesXY) )
    % calculate distance to all other nodes
    dvec = (nodesXY(n,:) - nodesXY);
    d = sqrt(dvec(:,1).^2 + dvec(:,2).^2);
    % find similar nodes
    ind = find(d <= Rthresh);
    % the ind will always contain the node n (distance zero)    
    if ( length(ind) >= 2 )
       % calculate a new node position to approximate this group
        meanPos = mean(nodesXY(ind,:),1);
        % delete old nodes
        nodesXY(ind,:) = [];
        % add the new node
        nodesXY(end+1,:) = meanPos;
        % reset and begin searching again from the beginning..  
        n = 1;
    else      
        n = n +1;
    end
end

% truncate nodes to a box of size dim x dim
truncInd = [];
for i = 1:1:length(nodesXY)
    if ( ( abs(nodesXY(i,1)) > dim/2 ) || ( abs(nodesXY(i,2)) > dim/2 )  )
       truncInd = [truncInd i]; 
    end
end
nodesXY(truncInd,:) = [];

% the last step is to remove any nodes that are solitary with respect 
% to the dx*edgeFactor criteria
edgelessNodes = [];
for i = 1:1:length(nodesXY)
    % calculate distance to all other nodes
    dvec = (nodesXY(i,:) - nodesXY);
    d = sqrt(dvec(:,1).^2 + dvec(:,2).^2);
    ind = find(d <= dx*edgeFactor);
    if ( length(ind) == 1 )
        edgelessNodes = [edgelessNodes i];
    end
end
% remove the edgless nodes
nodesXY(edgelessNodes,:) = [];

end