



if (plotFlag)
    figure(fig)
    imagesc('XData',[xmin xmax],'YData',[ymin ymax],'CData',Ac)
    set(gca,'YDir','Normal')
    hold on;
    axis equal;
    plot(xbar,ybar,'mo','linewidth',2);
    plot(xs,ys,'c+','linewidth',2)
end