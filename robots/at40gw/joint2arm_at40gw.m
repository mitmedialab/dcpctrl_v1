function X = joint2arm_at40gw( Q )
%JOINT2ARM_AT40GW Returns cartesian positions of all joint locations from
%   the base to the end effector
% INPUTS:
%   Q - Mx4 vector of joint positions in degrees and mm, M is number of
%   frames
% OUTPUTS:
%   X - 6x4xM matrix of joint positions in mm

pe = [1; 0; 0; 0];

% TODO: Condense using arrayfun perhaps
for i = 1:size(Q,1)
    fk1 = fkine_j1_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    fk2 = fkine_j2_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    fk3 = fkine_j3_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    fk4 = fkine_j4_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    fk5 = fkine_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));

    X0 = [pe fk1*pe fk2*pe fk3*pe fk4*pe fk5*pe];

    X(:,:,i) = X0(2:4,:)';
end

end

