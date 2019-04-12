function status = targIsInFOV(targXY, xk, params)
for i = 1:1:params.N
    xi = [ xk(3*i-2); xk(3*i-1); xk(3*i)];
    agentX(i) = xi(1);
    agentY(i) = xi(2);
end
d2 = (targXY(1) - agentX).^2 + (targXY(2) - agentY).^2;
status = any( d2 <= params.Rsense^2 );
end