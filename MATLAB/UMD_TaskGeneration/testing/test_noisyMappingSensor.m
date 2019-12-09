clear; close all; clc;

nz = 15;
ng = nz;
mzRange = [0.5 2 3.5];
mgRange = [0.5 2 3.5];
np = 50;
k = 1;
for w = 1:1:length(mzRange)
    mz = mzRange(w);
    for y = 1:1:length(mgRange)
        mg = mgRange(y);
        % create sensor profiles
        g = linspace(-3,mg+3,ng);
        z = linspace(-3,mz+3,nz);
        
        for l = 1:1:ng
            g_V(l) = exp(-(g(l))^2/2);
            g_UO(l) = exp(-(g(l)-mg)^2/2);
        end
        g_V = g_V ./ sum(g_V);
        g_UO = g_UO ./ sum(g_UO);
        
        for q = 1:1:nz
            z_VU(q) = exp(-(z(q))^2/2);
            z_O(q) = exp(-(z(q)-mz)^2/2);
        end
        z_VU = z_VU ./ sum(z_VU);
        z_O = z_O ./ sum(z_O);
        
        
        oRange = linspace(0.01,0.99,np);
        vRange = linspace(0.01,0.99,np);
        for i = 1:1:length(vRange)
            i
            for j = 1:1:length(oRange)
                v = vRange(i);
                o = oRange(j);
                u = 1 - v - o;
                if (u >= 0)
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
                        end
                    end
                    I_C_GZ(i,j) = H_C - H_C_GZ;
                    if ( isinf(I_C_GZ(i,j)) || I_C_GZ(i,j) > H_C )
                        I_C_GZ(i,j) = NaN;
                    end
                else
                    I_C_GZ(i,j) = NaN;
                end
            end
        end
        
        figure(1);
        subplot(3,3,k)        
        %imagesc('XData',vRange,'YData',oRange,'CData',I_C_GZ)
        
        imagesc('XData',vRange,'YData',oRange,'CData',I_C_GZ,'AlphaData',~isnan(I_C_GZ))
        hold on;
        contour(vRange,oRange,I_C_GZ,10,'LineColor','k','linewidth',1)
        xlabel('o')
        ylabel('v')
        set(gca,'FontSize',16)
        colorbar;

        %axis off;
        caxis([0 log2(3)])

        plot([0 1],[0 0]','k-','linewidth',3);
        plot([0 1],[1 0]','k-','linewidth',3);
        plot([0 0],[0 1]','k-','linewidth',3);        
        set(gca,'YDir','normal')
        titleString = sprintf('mz = %3.2f, mg = %3.2f',mzRange(w),mgRange(y));
        title(titleString);
        k = k + 1;
        
    end
end

subplot(3,3,7)
axis on;
xlabel('o')
ylabel('v')
set(gca,'FontSize',16)
% hp4 = get(subplot(3,3,6),'Position');
% colorbar('Position', [hp4(1)+hp4(3)+0.01  hp4(2)  0.1  hp4(2)+hp4(3)*2.1])