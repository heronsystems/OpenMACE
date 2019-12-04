% Test script for sending API packets from the coordinator to remote Xbee
% devices
% October, 2019. Sheng Cheng

% c1 = uisetcolor;
% color1 = c1*255;
% 
% c2 = uisetcolor;
% color2 = c2*255;
% 
% c3 = uisetcolor;
% color3 = c3*255;
% 
% c4 = uisetcolor;
% color4 = c4*255;


% Define some colors
color1 = [10 0 0]; color2 = color1; color3 = color1; test = [0 50 50];
RIGHT = [0 255 0];
LEFT = [255 0 0];

% Messages to be sent
receiver = [];
color = [test test RIGHT RIGHT test test LEFT LEFT test test LEFT LEFT test test RIGHT RIGHT];
status = [0];

msg = [color];
% generate the packet
APIpacket = ZigBeePacket(1,msg);

% send
serialMsg = sscanf(APIpacket, '%2x');

% open a serial port
s = serialport('/dev/ttyUSB0',9600);

write(s, serialMsg, 'uint8');

delete(s);


function msg = color2Hex(color)
    temp = dec2hex(uint8(color));
    msg = temp(:)';

end