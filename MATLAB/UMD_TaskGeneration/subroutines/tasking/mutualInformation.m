function I = mutualInformation(m, V, U, O, nz)
zi = linspace(-3,m+3,nz);
gi = exp(-zi.^2/2);
gi = gi ./ sum(gi); % normalize
hi= exp(-(zi-m).^2/2);
hi = hi ./ sum(hi); % normalize
H_C = cellStateEntropy(V,U,O);
if (U < 0.001) % apply a tolerance
    term2 = 0;
else
    term2 = -sum(gi*U.*log2(gi*(U)./(gi*U + hi*O)));
end
if (O < 0.001) % apply a tolerance
    term3 = 0;
else
    term3 = -sum(hi*O.*log2(hi*(O)./(gi*U + hi*O)));
end
I = H_C - ( term2 + term3 );

% apply a tolerance
if I < 0.001
    I = 0;
end
end