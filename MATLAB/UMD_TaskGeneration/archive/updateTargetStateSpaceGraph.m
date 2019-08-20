function [targetStateSpaceGraph,Mpc2s, Mc, Mp] = updateTargetStateSpaceGraph(targetStateSpaceGraph, exploredGraph, Mpc2s, Mc, Mp, discoveredNodes)

for idNew = discoveredNodes'    
    % add new target states
    N = neighbors(exploredGraph,idNew);
    
    % add rest-state
    NodeProps = table(idNew,idNew,'VariableNames',{'prevNode','curNode'});
    targetStateSpaceGraph = addnode(targetStateSpaceGraph, NodeProps);
    Mpc2s(idNew,idNew) = numnodes(targetStateSpaceGraph);
    Mc(idNew,numnodes(targetStateSpaceGraph)) = 1;
    Mp(idNew,numnodes(targetStateSpaceGraph)) = 1;
    % add outgoing/incoming states
    for i = 1:1:length(N)
        NodeProps = table(idNew,N(i),'VariableNames',{'prevNode','curNode'});
        targetStateSpaceGraph = addnode(targetStateSpaceGraph, NodeProps);
        Mpc2s(idNew,N(i)) = numnodes(targetStateSpaceGraph);
        Mp(idNew,numnodes(targetStateSpaceGraph)) = 1;
        Mc(N(i),numnodes(targetStateSpaceGraph)) = 1;
        
        NodeProps = table(N(i),idNew,'VariableNames',{'prevNode','curNode'});
        targetStateSpaceGraph = addnode(targetStateSpaceGraph, NodeProps);
        Mpc2s(N(i),idNew) = numnodes(targetStateSpaceGraph);        
        Mp(N(i),numnodes(targetStateSpaceGraph)) = 1;
        Mc(idNew,numnodes(targetStateSpaceGraph)) = 1;
    end
end


    
for idNew = discoveredNodes' 
    % define case 1a edges
    v = idNew; % for convenience
    Nv = neighbors(exploredGraph,v);
    if ( ~isempty(Nv) )
        for y = Nv'
            Ny = neighbors(exploredGraph,y);
            if ( ~isempty(Ny) )
                for z = Ny'
                    n1 = Mpc2s(v,y);
                    n2 = Mpc2s(y,z);
                    
                    targetStateSpaceGraph = addedge(targetStateSpaceGraph, n1, n2);
                end
            end
            % special case z = y
            z = y;
            n1 = Mpc2s(v,y);
            n2 = Mpc2s(y,z);
            targetStateSpaceGraph = addedge(targetStateSpaceGraph, n1, n2);
        end
    end
    
    % define case 1b edges
    if ( ~isempty(Nv) )
        for z = Nv'
            n1 = Mpc2s(v,v);
            n2 = Mpc2s(v,z);
            targetStateSpaceGraph = addedge(targetStateSpaceGraph, n1, n2);
        end
    end
    % 1b special case z = v;
    n1 = Mpc2s(v,v);
    n2 = Mpc2s(v,v);
    targetStateSpaceGraph = addedge(targetStateSpaceGraph, n1, n2);
    
    % define case 2a edges
    if ( ~isempty(Nv) )
        for x = Nv'
            Nx = neighbors(exploredGraph,x);
            if ( ~isempty(Nx) )
                for y = Nx'
                    if ( any(Nv==y) && y ~= v  )
                        n1 = Mpc2s(x,y);
                        n2 = Mpc2s(y,v);
                        targetStateSpaceGraph = addedge(targetStateSpaceGraph, n1, n2);
                    end
                end
            end
        end
    end
    
    % define case 2b edges
    if ( ~isempty(Nv) )
        for x = Nv'
            n1 = Mpc2s(x,x);
            n2 = Mpc2s(x,v);
            targetStateSpaceGraph = addedge(targetStateSpaceGraph, n1, n2);
        end
    end
    
    % define case 2c edges
    if ( ~isempty(Nv) )
        for x = Nv'
            for z = Nv'
                n1 = Mpc2s(x,v);
                n2 = Mpc2s(v,z);
                targetStateSpaceGraph = addedge(targetStateSpaceGraph, n1, n2);
            end
        end
    end
    
    % define case 3a edges
    if ( ~isempty(Nv) )
        for y = Nv'
            Ny = neighbors(exploredGraph,y);
            if ( ~isempty(Ny) )
                for x = Ny'
                    if ( ~any(x==Nv) && x ~= v )
                        n1 = Mpc2s(x,y);
                        n2 = Mpc2s(y,v);
                        targetStateSpaceGraph = addedge(targetStateSpaceGraph, n1, n2);
                    end
                end
            end
        end
    end
    
end

end