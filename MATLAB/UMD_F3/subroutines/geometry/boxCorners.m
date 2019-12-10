function boxXY = boxCorners(x,y,length,width,angle)

boxX = [0 1 1 0]*length;
boxY = [1 1 0 0]*width;
R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
boxXY = zeros(4,2);
for i = 1:1:4
   pt = [boxX(i) boxY(i)]';
   ptrot = R*pt;
   boxXY(i,1) = ptrot(1) + x;
   boxXY(i,2) = ptrot(2) + y;
end
end