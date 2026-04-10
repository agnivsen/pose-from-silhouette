% ============================
%% postelToRotationMatrix: converts Postel projection of unit quaternion back to 
% unit quaternion and rotation matrix
%
% Input: alpha, rVector
% Output: R, quat
% ============================
%
function [R, quat] = postelToRotationMatrix(alpha, rVector)
    assert(numel(rVector) == 3, '<rVector> must be a column vector [1 x 3]');
    assert(abs(alpha) < pi, '<alpha> must be in [-pi, pi]');

    scaledRvec = sin(alpha/2).*rVector;
    quat = quaternion(cos(alpha/2), scaledRvec(1), scaledRvec(2), scaledRvec(3));

    R = quat2rotm(quat);
end