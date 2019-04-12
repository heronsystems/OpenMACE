function H = gridSensorEntropy(p,q,r)
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