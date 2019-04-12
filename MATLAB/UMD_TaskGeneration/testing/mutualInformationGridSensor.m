function I = mutualInformationGridSensor(p, q, r)
H_C = cellStateEntropy(p,q,r);
H_CG = q.*log2((q+r)./q) + r*log2((q+r)./r);
I = H_C - H_CG;
end