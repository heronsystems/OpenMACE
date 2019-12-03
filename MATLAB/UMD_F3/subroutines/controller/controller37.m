% Formation controller adopted from implmentation by Charles Flanagan of 
% Dr. Derek A. Paley's paper:
% Sepulchre, Rodolphe, Derek A. Paley, and Naomi Ehrich Leonard. "Stabilization of planar collective motion: All-to-all communication." IEEE Transactions on Automatic Control 52.5 (2007): 811-824.
%
% This controller follows Eq. 37 in the paper.

% Inputs:
%   - N: number of agents
%   - position: N x 2 vector, each row of which contains the position of an
%   agent in a plane
%   - agnel: N x 1 vector, each row of which contains the angle of an agent
%   in radian
%   - K: gain
%   - wo: 1/abs(wo) is the radius of the formation circle

% Output:
%   - u: N x 1 vector, each row of which contains the angular velocity of
%   an agent in rad/s

% Sheng Cheng, Nov. 2019

function u = controller37(position,angle,N,K,wo)
% Calculate sum for kU1
% for i=1:N
%     sumU1(i) = 0;
% end
sumU1 = zeros(1,N);
for k=1:N
    for l=1:N
        sumU1(k) = sumU1(k) + sin(angle(l)-angle(k));
    end
end

% Calculate sum for UMN
M=N;
Km=K;
sumMN = zeros(1,N);
% for i=1:N
%     sumMN(i) = 0;
% end

for k=1:N
    for l=1:N
        for mm=1:N
            sumMN(k) = sumMN(k) + (Km/mm)*sin(mm*(angle(k)-angle(l)));
        end
    end
end

sum_rk=0;

for j=1:N
    sum_rk=sum_rk + position(1,j)+1i*position(2,j);
end

R = (1/N)*sum_rk;

for j=1:N
    r_tilda(j) = position(1,j)+1i*position(2,j) - R;
end

for ll=1:N
    u(ll) = wo*(1+K*real(dot(r_tilda(ll),exp(1i*angle(ll))))) + sumMN(ll);% -sumU1(ll);
end

end