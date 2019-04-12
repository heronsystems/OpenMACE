function H = gridSensorNoisyEntropy(p,q,r)

zi = linspace(-3,m+3,nz);
gi = exp(-zi.^2/2);
gi = gi ./ sum(gi); % normalize
hi= exp(-(zi-m).^2/2);
hi = hi ./ sum(hi); % normalize
p = gi*(p+q) + hi*r;
H = sum(-p.*log2(p));


    if ( p == 0 )
        t1 = 0;
    else
        t1 = -p*log2(p);
    end
    if ( q + r == 0 )
        t2 = 0;
    else
        t2 =  -(q+r)*log2(q+r);
    end
    H = t1 + t2;
end