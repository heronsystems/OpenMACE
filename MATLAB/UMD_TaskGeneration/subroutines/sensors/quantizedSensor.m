function [val] = quantizedSensor(m, levels, stimulus)

% output vals
rangeStdev = 3;
z = linspace(-rangeStdev,m+rangeStdev,levels);

% continuous output
if ( stimulus ) % signal
    signal = randn() + m;
else % noise
    signal = randn();
end

% quantize
val = interp1(z,[1:1:levels],signal,'nearest','extrap');

% if ( signal <= z(1) )
%     val = 1;
% else
%     val = 1;
%     for i = 1:1:levels
%         if signal > z(i)
%             val = i;
%         end
%     end
% %     while ( signal > z(val) && val <= levels )
% %         val = val + 1;
% %     end
% end
% val

end
