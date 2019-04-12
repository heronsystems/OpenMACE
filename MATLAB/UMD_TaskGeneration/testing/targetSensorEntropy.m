function H = targetSensorEntropy(p,q,r,m,nz)
zi = linspace(-3,m+3,nz);
gi = exp(-zi.^2/2);
gi = gi ./ sum(gi); % normalize
hi= exp(-(zi-m).^2/2);
hi = hi ./ sum(hi); % normalize
p = gi*(p+q) + hi*r;
H = sum(-p.*log2(p));
end