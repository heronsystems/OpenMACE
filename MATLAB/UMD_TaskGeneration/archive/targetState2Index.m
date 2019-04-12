function s = targetState2Index(Mc, Mp, c, p)
for i = 1:1:Ns
    if ( Mc(i,initCur) == 1 && Mp(i,initPrev) == 1 )
        s = i;
    end
end
end