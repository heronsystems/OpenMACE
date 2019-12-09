function I_C_GZ = mutualInformationMappingTargetOld(V, U, O, nz, ng, z_VU, z_O, g_V, g_UO)

numRows = size(V,1);
numCols = size(V,2);

for i = 1:1:numRows
    for j = 1:1:numCols
        v = V(i,j);
        u = U(i,j);
        o = O(i,j);
        
        H_C = cellStateEntropy(u,v,o);
        H_C_GZ = 0;
        for q = 1:1:nz
            for l = 1:1:ng
                kv = v*z_VU(q)*g_V(l);
                ku = u*z_VU(q)*g_UO(l);
                ko = o*z_O(q)*g_UO(l);
                p_g_z = v*g_V(l)*z_VU(q) + u*g_UO(l)*z_VU(q) + o*g_UO(l)*z_O(q);
                term1 = kv*log2(kv/p_g_z);
                term2 = ko*log2(ko/p_g_z);
                term3 = ku*log2(ku/p_g_z);
                H_C_GZ = H_C_GZ - (term1 + term2 + term3);
                kvMat(q,l) = kv;
                kuMat(q,l) = ku;
                koMat(q,l) = ko;
                p_g_zMat(q,l) = p_g_z;
                term1Mat(q,l) = term1;
                term2Mat(q,l) = term2;
                term3Mat(q,l) = term3;
            end
        end
       
%         figure;
%         subplot(3,1,1);
%         imagesc(term1Mat)
%         subplot(3,1,2);
%         imagesc(term2Mat)
%         subplot(3,1,3);
%         imagesc(term3Mat)
        
%         figure;
%         subplot(4,1,1);
%         imagesc(kvMat)
%         subplot(4,1,2);
%         imagesc(kuMat)
%         subplot(4,1,3);
%         imagesc(koMat)
%         subplot(4,1,4);
%         imagesc(p_g_zMat)        

        I_C_GZ(i,j) = H_C - H_C_GZ;
        if ( isinf(I_C_GZ(i,j)) )
            I_C_GZ(i,j) = NaN;
            error('I_C_GZ = Inf');
        elseif( I_C_GZ(i,j) < 0 )
            error('I_C_GZ = 0');
        end
        
    end
end



%     zi = linspace(-3,m+3,nz);
%     gi = exp(-zi.^2/2);
%     gi = gi ./ sum(gi); % normalize
%     hi= exp(-(zi-m).^2/2);
%     hi = hi ./ sum(hi); % normalize
%     H_C = cellStateEntropy(V,U,O);
%     if (U < 0.001) % apply a tolerance
%         term2 = 0;
%     else
%         term2 = -sum(gi*U.*log2(gi*(U)./(gi*U + hi*O)));
%     end
%     if (O < 0.001) % apply a tolerance
%         term3 = 0;
%     else
%         term3 = -sum(hi*O.*log2(hi*(O)./(gi*U + hi*O)));
%     end
%     I = H_C - ( term2 + term3 );
%
%     % apply a tolerance
%     if I < 0.001
%         I = 0;
%     end
end