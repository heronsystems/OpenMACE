function [ Q ] = stateTransitionMatrix ( G_tss, probStopping, m , d  )

% Q is Ns x Ns matrix
% Entry (i,j) gives prob from state i to state j

Ns = numnodes(G_tss);
Q = zeros(Ns,Ns);
for si = 1:1:Ns
    Q = updateTransitionProbForState(si, Q, G_tss, probStopping, m, d);
    fprintf('Step %d of %d\n',si,Ns);
end


end
