function w = relWeightFun(x,m,d)
b = -1/d;
a = -m*(pi-b)*b/pi;
c = 1 + m + a/b;
w = a*(1./(x-b)) + c;
end