function [entropyMat, totalEntropy] = entropyMatrix(p, q, r)
% compute entropy of entire search grid
numRows = size(p,1);
numCols = size(p,2);


% 
entropyMat = zeros(numRows,numCols);
for j = 1:1:numRows
    for k = 1:1:numCols
        % this is required to avoid evaluating to nan or inf 
        if ( p(j,k) ~= 0 )
            term1 = -p(j,k)*log2(p(j,k));
        else
            term1 = 0;
        end
        if ( q(j,k) ~= 0 )
            term2 = -q(j,k)*log2(q(j,k));
        else
            term2 = 0;
        end
        if ( r(j,k) ~= 0 )
            term3 = -r(j,k)*log2(r(j,k));
        else
            term3 = 0 ;
        end
        entropyMat(j,k) = term1 + term2 + term3;
        if ( isnan(entropyMat(j,k)) )
            error('entropyMat(j,k) is NaN')
        elseif  ( isinf(entropyMat(j,k)) )
            error('entropyMat(j,k) is Inf')
        end
    end
end

% debug
% figure;
% subplot(4,1,1)
% imagesc(-p.*log2(p));
% axis equal; axis tight;
% title('-p*log2(p)'); colorbar; caxis([0 log2(3)])
% 
% subplot(4,1,2)
% imagesc(-q.*log2(q));
% axis equal; axis tight;
% title('-q*log2(q)'); colorbar; caxis([0 log2(3)])
% 
% subplot(4,1,3)
% imagesc(-r.*log2(r));
% axis equal; axis tight;
% title('-r*log2(r)'); colorbar; caxis([0 log2(3)])
% 
% subplot(4,1,4)
% imagesc( entropyMat  );
% axis equal; axis tight; colorbar; caxis([0 log2(3)])
% title('-(p.*log2(p) + q.*log2(q) + r.*log2(r) )');

totalEntropy = sum(sum(entropyMat));
end