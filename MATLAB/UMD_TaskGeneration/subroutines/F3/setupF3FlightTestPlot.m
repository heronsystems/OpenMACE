function setupF3FlightTestPlot( runParams )

% Set up plotting
h=figure(1);
set(h, 'Position', [100, 100, 800, 800]);
subplot(2,1,1)
%ylim([0 2])
xlim([0 30])
xlabel('Time (sec.)')
ylabel('Altitude (m)')
hsp1 = get(gca, 'Position');
set(gca,'FontSize',16)
grid on;
hold on;

subplot(2,1,2) 
xlabel('X (m)')
ylabel('Y (m)')
set(gca,'FontSize',16)
grid on;
img = imread(runParams.movie.backgroundImgFile);
hold on;
xdataVec = [runParams.movie.backgroundImgBottomLeftCornerX runParams.movie.backgroundImgBottomLeftCornerX+runParams.movie.backgroundImgWidth];
ydataVec = [runParams.movie.backgroundImgBottomLeftCornerY runParams.movie.backgroundImgBottomLeftCornerY+runParams.movie.backgroundImgHeight];
image(flipud(img), 'XData', xdataVec, 'YData', ydataVec );
set(gca,'YDir','Normal')
plot(runParams.movie.perimX, runParams.movie.perimY, 'k-','Linewidth',2);
plot(runParams.movie.pole1x, runParams.movie.pole1y, 'k-','Linewidth',2);
plot(runParams.movie.pole2x, runParams.movie.pole2y, 'k-','Linewidth',2);
axis equal;
axis tight;
xlim([-65 35])
ylim([-25 20])
hsp2 = get(gca, 'Position');                     
set(gca, 'Position', [hsp2(1:3)  hsp1(4)])

end