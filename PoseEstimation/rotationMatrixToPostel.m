% ============================
%% rotationMatrixToPostel: converts rotation matrices to Postel projection coordinates
%
% Input: R \in SO(3)
% Output: alpha, rVec
% ============================
%
function [alpha, rVec] = rotationMatrixToPostel(R)
    Q = rotm2quat(R);
    alpha = 2*acos(Q(1));
    rVec = Q(2:4)./(sin(alpha/2));
end