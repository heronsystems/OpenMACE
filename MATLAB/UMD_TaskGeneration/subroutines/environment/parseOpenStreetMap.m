function [ways, LatRef, LongRef, xRef, yRef, utmzone] = parseOpenStreetMap(fileName)
disp('Parsing OpenStreetMap...')
% code adapted from openstreet map example
[parsed_osm, osm_xml] = parse_openstreetmap(fileName);
[bounds, node, way, ~] = assign_from_parsed(parsed_osm);
LatRef = mean(bounds(2,:));
LongRef = mean(bounds(1,:));
% LatRef = ( max(bounds(2,:)) -  min(bounds(2,:)))/2;
% LongRef = ( max(bounds(1,:)) -  min(bounds(1,:)))/2;
% define local origin in UTM coordinates
% Recall Lat indicates position north/south, Long is position east/west
% E.g., boston Lat 42.3601° N,  Long 71.0589° W
[xRef,yRef,utmzone] = deg2utm(LatRef, LongRef);
nodesXY = [];
ways = [];
k = 1;
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
            [xc,yc,~] = deg2utm(nd_coor(2,:),nd_coor(1,:));            
            ways{k} = [xc-xRef yc-yRef]; 
            k = k + 1;
        end
    end
end

end