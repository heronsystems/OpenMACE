function [h] = randomHeadingCW(h1, h2)
dth = angularDist(h1,h2);
h = mod ( h1 + rand()*dth , 2*pi );
end