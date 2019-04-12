function [ G_tss, Mp, Mc ] = environmentToStateSpace( G_env )
A_env = adjacency(G_env);
A_env_aug = A_env + eye(size(A_env));
N = numnodes(G_env);

nodeID = 1;
% define nodes in the state-space graph
for i = 1:1:N
    for j = 1:1:N
        if ( A_env_aug(i,j) == 1 )
            dx = G_env.Nodes.x(j) - G_env.Nodes.x(i);
            dy = G_env.Nodes.y(j) - G_env.Nodes.y(i);
            heading = atan2( dy , dx );
            nodeProp_i(nodeID) = i;
            nodeProp_j(nodeID) = j;
            nodeProp_h(nodeID) = heading;
            nodeID = nodeID + 1; 
        end
    end
    fprintf('Step %d of %d\n',i,N);
end
nodeIDvec = [1:1:nodeID-1];
Mp = sparse(nodeIDvec,nodeProp_i,1);
Mc = sparse(nodeIDvec,nodeProp_j,1);
disp('Nodes Defined')

nodeProps = table(nodeProp_i',nodeProp_j',nodeProp_h','VariableNames',{'prevNode','curNode','heading'});
A_tss = Mc*Mp';
G_tss = digraph(A_tss,nodeProps);
disp('Edges Defined')


end