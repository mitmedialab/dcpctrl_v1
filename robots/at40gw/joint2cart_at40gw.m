function X = joint2cart_at40gw( Q )
%JOINT2CART_AT40GW Converts AT40GW joint positions in joint-space to cartesian positions
%of the boom arm end-effector using forward kinematics, does NOT return
%intermediate joint positions
% INPUTS:
%   Q - Nx4 matrix, column per joint, must be in degrees and mm
% OUTPUTS:
%   X - Nx3 matrix, [X, Y, Z] cartesian positions of the end-effector of
%   the boom arm

% TODO: can we vectorize further?
X = zeros(size(Q,1), 3);
for i = 1:size(Q,1)
    fk = fkine_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    p = fk * [1; 0; 0; 0];
    X(i, :) = p(2:4)';
end

end

