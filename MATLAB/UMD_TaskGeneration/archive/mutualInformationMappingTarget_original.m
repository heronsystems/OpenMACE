function [H_C, I_C_GZ, totalEntropy] = mutualInformationMappingTarget_original(V, U, O, z_VU, z_O, g_V, g_UO)

numRows = size(V,1);
numCols = size(V,2);
tolZero = 0.01;
H_C = cellStateEntropyVectorized(V, U, O); 
for i = 1:1:numRows
    for j = 1:1:numCols
        if H_C(i,j) < tolZero
            I_C_GZ(i,j) = 0;
        else
            p_g_z = V(i,j)*(g_V'*z_VU) + U(i,j)*(g_UO'*z_VU) + O(i,j)*(g_UO'*z_O);
            p_g_z = p_g_z';
            
            kv = V(i,j)*(z_VU'*g_V);
            term1Mat = kv.*log2(kv./p_g_z);
            term1Mat( isnan(term1Mat) ) = 0;
            
            ku = U(i,j)*(z_VU'*g_UO);
            term2Mat = ku.*log2(ku./p_g_z);
            term2Mat( isnan(term2Mat) ) = 0;
            
            ko = O(i,j)*(z_O'*g_UO);
            term3Mat = ko.*log2(ko./p_g_z);
            term3Mat( isnan(term3Mat) ) = 0;
            
            % sum
            I_C_GZ(i,j) = H_C(i,j) + ( sum(sum(term1Mat+term2Mat+term3Mat)) );
        end
        
        % error checking
        if ( isinf(I_C_GZ(i,j)) )
            I_C_GZ(i,j) = NaN;
            error('I_C_GZ = Inf');
        elseif ( isnan(I_C_GZ(i,j)) )
            error('I_C_GZ = NaN')
        end
        if( I_C_GZ(i,j) <= -tolZero )
            error('I_C_GZ < 0');
        elseif ( -tolZero < I_C_GZ(i,j) < 0 )
            I_C_GZ(i,j) = 0;
        end
        if ( I_C_GZ(i,j) > H_C(i,j) )
           error('I_C_GZ(i,j) > H_C(i,j)');
        end
        
    end
end

% error check
if ( ~isreal(I_C_GZ) )
    error('I_C_GZ is not real');
end

totalEntropy = sum(sum(H_C));


end
