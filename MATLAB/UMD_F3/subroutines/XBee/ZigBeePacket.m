% ZigBeePacket sends msg to the ZigBee on the quadrotor with quadID
% quadID should be a number and msg should be a text string
% October, 2019. Sheng Cheng

function APIpacket = ZigBeePacket(quadID,msg)

% address on each quad
% The address below is associated with XBee module 2
ZigBeeAddress_64{1} = ['0013A200419B5B73'];
ZigBeeAddress_16{1} = ['DF23']; % it seems this address changes every time. Maybe consider change it to FFFE (broadcasting)

% turn the text msg to hex format
temp = dec2hex(uint8(msg))';
msgHex = temp(:)';

frameLength = length(msg) + 14;
% length breakdown: Frame type 1
%                   Frame ID 1
%                   64-bit address 8
%                   16-bit address 2
%                   Broadcast radius 1
%                   Options 1

StartDeliminater = '7E';
FrameType = '10';
FrameID = '01';
BroadcastRadius = '00';
Options = '00';

FrameData = [FrameType ... % Frame type
             FrameID ... % Frame ID
             ZigBeeAddress_64{quadID} ... % 64-bit destination address
             ZigBeeAddress_16{quadID} ... % 16-bit destination address
             BroadcastRadius ... % Broadcast radius
             Options ... % Options
             msgHex]; % msg
         
CheckSum = chksum(FrameData);

serialMsgRaw = [dec2hex(frameLength,4)... % Length
             FrameData ... % RF data
             CheckSum]; 
APIpacket = [];         

% check for escaped bytes: 7E 7D 11 13 in serialMsg 
for k = 1:length(serialMsgRaw)/2
    switch serialMsgRaw(2*k-1:2*k)
        case '7E' %, insert 0x7D and follow it with the byte to be escaped XOR’d with 0x20
            APIpacket = [APIpacket '7D' '5E'];
        case '7D'
            APIpacket = [APIpacket '7D' '5D'];
        case '11'
            APIpacket = [APIpacket '7D' '31'];
        case '13'
            APIpacket = [APIpacket '7D' '33'];
        otherwise
            APIpacket = [APIpacket serialMsgRaw(2*k-1:2*k)];
    end
end

% add StartDeliminator
APIpacket = ['7E' APIpacket];

