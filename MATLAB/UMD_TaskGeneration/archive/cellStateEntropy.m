function H = cellStateEntropy(p,q,r)
    if ( p == 0 )
        t1 = 0;
    else
        t1 = -p.*log2(p);
    end
    if ( q == 0 )
        t2 = 0;
    else
        t2 = -q.*log2(q);
    end
    if ( r == 0 )
        t3 = 0;
    else
        t3 = -r.*log2(r);
    end
    H = t1 + t2 + t3;
end