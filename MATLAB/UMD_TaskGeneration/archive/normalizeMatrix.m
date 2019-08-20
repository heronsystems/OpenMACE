function M = normalizeMatrix(M, lb, ub)
    minM = min(min(M));
    maxM = max(max(M));
    M = (M - minM)/(maxM - minM); % from 0 to 1
    M = lb + M*(ub-lb);
end