function I = mutualInformationTargetSensor(m, p, q, r, nz)
H_C = cellStateEntropy(p,q,r);
zi = linspace(-3,m+3,nz);
gi = exp(-zi.^2/2);
gi = gi ./ sum(gi); % normalize
hi= exp(-(zi-m).^2/2);
hi = hi ./ sum(hi); % normalize
if ( p == 0 )
    term1 = 0;
else
    term1 = p*gi.*log2((gi*(p+q)+hi*r)./(p*gi));
end
if ( q == 0 )
    term2 = 0;
else
    term2 = q*gi.*log2((gi*(p+q)+hi*r)./(q*gi));
end
if ( r == 0)
    term3 = 0;
else
    term3 = r*hi.*log2((gi*(p+q)+hi*r)./(r*hi));
end

H_CZ = sum(term1 + term2 + term3);
I = H_C - H_CZ;
end