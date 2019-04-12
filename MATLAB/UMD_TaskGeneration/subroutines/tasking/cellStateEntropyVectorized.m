function H = cellStateEntropyVectorized(V,U,O)
% term1
t1 = - V.*log2(V);
t1(V==0) = 0;

% term2
t2 = - U.*log2(U);
t2(U==0) = 0;

% term3
t3 = - O.*log2(O);
t3(O==0) = 0;

H = t1 + t2 + t3;


H( imag(H)~=0 ) = 0;
% if ( ~isreal(H) )
%     error('H is not real');
% end
end