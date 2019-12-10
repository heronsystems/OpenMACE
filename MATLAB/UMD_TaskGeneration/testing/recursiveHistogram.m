function [edgeHist, peaks] = recursiveHistogram(edgeHist, numPeaks, peaks, peakMags, dth, newDirs)
minPeak = min(peakMags);
for i = 1:1:length(newDirs)
   ind = floor(newDirs(i) / dth);
   edgehist(ind) = edgehist(ind) + 1;
   if ( edgehist(ind) > minPeak ) 
        for j = 1:1:length(peakMags)
            % peaks are sorted by decreasing peakMags
            if ( edgehist(ind) > peakMags(j) )
                
            end
        end
   end
end

end