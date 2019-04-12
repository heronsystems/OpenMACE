function [H_C, I_C_GZ, totalEntropy] = mutualInformationMappingTarget(V, U, O, z_VU, z_O, g_V, g_UO)

numRows = size(V,1);
numCols = size(V,2);

% % debug code for comparing entropy calculations
% %-------------------------------------------
% %This debug code shows that vector vs. element-wise log2 operator can have
% %differences of up to 0.3 bits when V,U,O entry is near zero
% disp('H_C_vector = cellStateEntropyVectorized(V, U, O);');
% tic;
% H_C_vector = cellStateEntropyVectorized(V, U, O);
% toc;
% disp('H_C_scalar(i,j) = cellStateEntropyVectorized(V(i,j), U(i,j), O(i,j));');
% tic;
% for i = 1:1:numRows
%     for j = 1:1:numCols
%         H_C_scalar(i,j) = cellStateEntropyVectorized(V(i,j), U(i,j), O(i,j));
%     end
% end
% toc;
% figure;
% subplot(2,3,1)
% imagesc(V); colorbar; title('V');
% subplot(2,3,2)
% imagesc(U); colorbar; title('U');
% subplot(2,3,3)
% imagesc(O); colorbar; title('O');
% subplot(2,3,4)
% imagesc(H_C_vector); colorbar; title('H C vector');
% subplot(2,3,5)
% imagesc(H_C_scalar); colorbar; title('H C scalar');
% subplot(2,3,6)
% imagesc(H_C_vector - H_C_scalar); colorbar; title('H_C diff');

% % debug code for comparing info calculations
% % -------------------------------------------
% tic;
% tolZero = 0.1;
% nz = length(z_VU);
% ng = length(g_V);
% for i = 1:1:numRows
%     for j = 1:1:numCols
%         v = V(i,j);
%         u = U(i,j);
%         o = O(i,j);
%         H_C = cellStateEntropy(v,u,o);
%         if H_C < tolZero
%             I_C_GZ(i,j) = 0;
%         else
%             H_C_GZ = 0;
%             for q = 1:1:nz
%                 for l = 1:1:ng
%                     p_g_z = v*g_V(l)*z_VU(q) + u*g_UO(l)*z_VU(q) + o*g_UO(l)*z_O(q);
%                     if ( V(i,j) ~= 0 )
%                         kv = v*z_VU(q)*g_V(l);
%                         term1 = kv*log2(kv/p_g_z);
%                     else
%                         term1 = 0;
%                     end
%                     if ( U(i,j) ~= 0 )
%                         ku = u*z_VU(q)*g_UO(l);
%                         term2 = ku*log2(ku/p_g_z);
%                     else
%                         term2 = 0;
%                     end
%                     if ( O(i,j) ~= 0 )
%                         ko = o*z_O(q)*g_UO(l);
%                         term3 = ko*log2(ko/p_g_z);
%                     else
%                         term3 = 0;
%                     end
%                     H_C_GZ = H_C_GZ - (term1 + term2 + term3);
%                 end
%             end
%             I_C_GZ(i,j) = H_C - H_C_GZ;
%         end
%         if ( isinf(I_C_GZ(i,j)) )
%             I_C_GZ(i,j) = NaN;
%             error('I_C_GZ = Inf');
%         elseif ( isnan(I_C_GZ(i,j)) )
%             error('I_C_GZ = NaN')
%         end
%         if( I_C_GZ(i,j) <= -tolZero )
%             error('I_C_GZ < 0');
%         elseif ( -tolZero < I_C_GZ(i,j) < 0 )
%             I_C_GZ(i,j) = 0;
%         end
%         
%     end
% end
% disp('scalar');
% toc;
% I_C_GZ_scalar = I_C_GZ;
% % vector form
% tic;
% H_C = cellStateEntropyVectorized(V, U, O);
% for i = 1:1:numRows
%     for j = 1:1:numCols
%         if H_C(i,j) < tolZero
%             I_C_GZ(i,j) = 0;
%         else
%             p_g_z = V(i,j)*(g_V'*z_VU) + U(i,j)*(g_UO'*z_VU) + O(i,j)*(g_UO'*z_O);
%             p_g_z = p_g_z';
%             kv = V(i,j)*(z_VU'*g_V);
%             term1Mat = kv.*log2(kv./p_g_z);
%             term1Mat(V==0) = 0;
%             ku = U(i,j)*(z_VU'*g_UO);
%             term2Mat = ku.*log2(ku./p_g_z);
%             term2Mat(U==0) = 0;
%             ko = O(i,j)*(z_O'*g_UO);
%             term3Mat = ko.*log2(ko./p_g_z);
%             term3Mat(O==0) = 0;
%             % sum
%             I_C_GZ(i,j) = H_C(i,j) + ( sum(sum(term1Mat+term2Mat+term3Mat)) );
%         end
%         
%         % error checking
%         if ( isinf(I_C_GZ(i,j)) )
%             I_C_GZ(i,j) = NaN;
%             error('I_C_GZ = Inf');
%         elseif ( isnan(I_C_GZ(i,j)) )
%             error('I_C_GZ = NaN')
%         end
%         if( I_C_GZ(i,j) <= -tolZero )
%             error('I_C_GZ < 0');
%         elseif ( -tolZero < I_C_GZ(i,j) < 0 )
%             I_C_GZ(i,j) = 0;
%         end
%         
%     end
% end
% disp('vector');
% toc;
% 
% I_C_GZ_vector = I_C_GZ;
% figure;
% subplot(1,3,1)
% imagesc(I_C_GZ_scalar); colorbar;
% title('I_C_GZ scalar');
% subplot(1,3,2)
% imagesc(I_C_GZ_vector); colorbar;
% title('I_C_GZ vector');
% subplot(1,3,3)
% imagesc(I_C_GZ_vector - I_C_GZ_scalar); colorbar;
% title('I_C_GZ diff');
% 
% 1;

%% actual implementation
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
            %term1Mat( isinf(term1Mat) ) = 0;
            
            ku = U(i,j)*(z_VU'*g_UO);
            term2Mat = ku.*log2(ku./p_g_z);
            term2Mat( isnan(term2Mat) ) = 0;
            %term2Mat( isinf(term2Mat) ) = 0;
            
            ko = O(i,j)*(z_O'*g_UO);
            term3Mat = ko.*log2(ko./p_g_z);
            term3Mat( isnan(term3Mat) ) = 0;
            %term3Mat( isinf(term3Mat) ) = 0;
            
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
%I_C_GZ = I_C_GZ';


% error check
if ( ~isreal(I_C_GZ) )
    error('I_C_GZ is not real');
end

totalEntropy = sum(sum(H_C));


end
