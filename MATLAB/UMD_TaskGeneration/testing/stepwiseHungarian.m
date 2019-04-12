% this is the test script for the step-wise Hungarian
clear all;

n = 4;
% rng(5);
% 
% G = round(rand(n));
% G = triu(G) + triu(G,1)';
% G = G - diag(diag(G));
% 
% G = graph(G);

G = graph(delsq(numgrid('S',n+2)),'omitselfloops');

G.Edges.Weight = round(10*rand(numedges(G),1));

% k = number of bundles
k = 2;

% q = bundle size
q = 5;

% initial nodes
A = randperm(n,k)';
Ahist = A;

% initial target nodes
T = union(neighbors(G,A(1)),neighbors(G,A(2)));
Thist = T;

% placeholder for previously traversed edge
traversedEdge = [];

for p = 1:q
    % construct the utility matrix
    utility = zeros(length(A),length(T));
    for i = 1:length(A)
        for j = 1:length(T)
            if ~~findedge(G,A(i),T(j))
                if sum(find(findedge(G,A(i),T(j))==traversedEdge))
                    utility(i,j) = 0;
                else
                    utility(i,j) = -G.Edges.Weight(findedge(G,A(i),T(j)));
                end
            else
                utility(i,j) = 0;
            end
        end
    end
    Anew = T(munkres(utility));
    Tnew = union(neighbors(G,Anew(1)),neighbors(G,Anew(2)));
    Ahist = [Ahist Anew];
%     Thist = [Thist Tnew];
    
    % record the traversed edges
    traversedEdge = union(traversedEdge,findedge(G,A,Anew));
        
    A = Anew;
    T = Tnew;
    
end

% global optimal solution =========
% generate an alternative graph because the weights on the edges will
% change
% =================================

graph_h = plot(G,'EdgeLabel',G.Edges.Weight);
title(['Starting node ' num2str(Ahist(1,1)) '(green) and ' num2str(Ahist(2,1)) '(red)']);

% highlight the path
highlight(graph_h,Ahist(1,:),'NodeColor','g','EdgeColor','g');
highlight(graph_h,Ahist(2,:),'NodeColor','r','EdgeColor','r');


