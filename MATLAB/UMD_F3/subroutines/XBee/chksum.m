% This function calculates teh Checksum byte in the ZigBee API frame
% message
% The input msg is a text string representing the HEX frame data
% The output CheckSum is a text string representing the HEX checksum result
% October, 2019. Sheng Cheng
function CheckSum = chksum(msg)

totalBytes = length(msg)/2;

sum = 0;
for k = 1:totalBytes
    sum = sum + hex2dec(msg(2*k-1:2*k));
end
sumHex = dec2hex(sum,2);

temp = hex2dec('FF') - hex2dec(sumHex(end-1:end));
CheckSum = dec2hex(temp,2);