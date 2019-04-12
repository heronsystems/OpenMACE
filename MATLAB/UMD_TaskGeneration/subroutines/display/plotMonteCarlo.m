function plotMonteCarlo(varargin)
% x, data, c, alpha
% x, data, c1, c2, c3, alpha
if (length(varargin) == 5)
    x = varargin{1};
    data = varargin{2};
    c = varargin{3};
    alpha = varargin{4};
    fig = varargin{5};
elseif ( length(varargin) == 7 )
    x = varargin{1};
    data = varargin{2};
    c1 = varargin{3};
    c2 = varargin{4};
    c3 = varargin{5};    
    alpha = varargin{6}; 
    figh = varargin{7};
else
    error('incorrect arguments')
end
figure(figh);
dataMean = meanOmitNaN(data,1);
dataStdev = stdevOmitNaN(data,1);
if ( length(varargin) == 5)
    plot(x,dataMean,[c '-'],'linewidth',1);
    hold on;
    h = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],c,'EdgeColor','none');
elseif ( length(varargin) == 7 )
    plot(x,dataMean,'Color',[c1 c2 c3],'linewidth',1);
    hold on;
    h = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],[c1 c2 c3],'EdgeColor','none');
end
set(h,'FaceAlpha',alpha);

end