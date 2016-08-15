function [ d_raw, err ] = djoint2draw_j1( robot, the, d_the )
%DJOINT2DRAW_J1 Map J1 angular velocity to sensor velocity

%{
    [ d_raw, err ] = djoint2draw_j1( robot, the, d_the )
    Julian Leland, MIT Media Lab, 2016-06-22
    For more details, see velmapgen.m
    Note that the is not used - J1 sensor velocity isn't a function of
    angle.
%}

% 737,280 counts/revolution on J1

d_raw = d_the .* (1/360) .* robot.Joint(1).PosSensParams.CountsRev; % Deg/sec * rev/deg * count/rev = count/sec

end

