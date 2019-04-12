function I = mutualInformation(m, p, q, r, nz)
zi = linspace(-3,m+3,nz);
gi = exp(-zi.^2/2);
gi = gi ./ sum(gi); % normalize
hi= exp(-(zi-m).^2/2);
hi = hi ./ sum(hi); % normalize
H_C = cellStateEntropy(p,q,r);
if (q == 0)
    term2 = 0;
else
    term2 = -sum(gi*q.*log2(gi*(q)./(gi*q + hi*r)));
end
if (r == 0)
    term3 = 0;
else
    term3 = -sum(hi*r.*log2(hi*(r)./(gi*q + hi*r)));
end
I = H_C - ( term2 + term3 );
end