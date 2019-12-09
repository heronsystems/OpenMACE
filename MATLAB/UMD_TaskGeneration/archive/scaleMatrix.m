function M = scaleMatrix(M)
    maxM = max(max(M));
    minM = min(min(M));
    M = (M-minM)./(maxM-minM);
end