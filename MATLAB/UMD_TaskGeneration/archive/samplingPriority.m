function sp = samplingPriority( swarmWorld )


sp = swarmWorld.O ./ swarmWorld.U;
[row, col] =  find( isnan(sp)==1 );
% 
% figure;
% subplot(5,1,1);
% imagesc(swarmWorld.priorR);
% subplot(5,1,2);
% imagesc(swarmWorld.priorQ);
% subplot(5,1,3);
% imagesc(sp);

if ( ~isempty(row) )
for i = 1:1:length(row)
        sp(row(i),col(i)) = 0;
end
end

% subplot(5,1,4);
% imagesc(sp);
% % subplot(5,1,5);
% imagesc(swarmWorld.samplingPriority);


end