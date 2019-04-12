function S = findTargStatesInView(Mc, nodesInView)

N = length(nodesInView);
S = [];
for i = 1:1:N
    n = nodesInView(i);
    % get the states with 'current node' corresponding to n
    Sn = find( Mc(n,:) == 1 )';
    S = [S; Sn];
end


end
