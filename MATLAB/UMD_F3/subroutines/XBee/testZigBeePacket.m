% Test script for sending API packets from the coordinator to remote Xbee
% devices
% October, 2019. Sheng Cheng

% Sample message
msg = 'Hello World! - from CDCL';

% generate the packet
APIpacket = ZigBeePacket(1,msg);

% send
serialMsg = sscanf(APIpacket, '%2x');

% open a serial port
s = serialport('/dev/ttyUSB0',9600);

write(s, serialMsg, 'uint8');

delete(s);
