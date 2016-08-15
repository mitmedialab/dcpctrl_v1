function [ d_raw, err ] = djoint2draw_j3( robot, the, d_the )
%DJOINT2DRAW_J3 Map J3 angular velocity to sensor velocity as a function of
%J3 angle.

% WARNING: The top portion of code is no longer used in an attempt to
% linearly approximate the surface. Will revisit this decision in the
% future.

%{
    [ d_raw, err ] = djoint2draw_j3( robot, the, d_the )
    Julian Leland, MIT Media Lab, 2016-06-22
    For more details, see velmapgen.m
%}

% Define max and min raw velocities that are available
d_rawmax = robot.Joint(3).VelMap.PosKnee; % Maximum raw velocity that system can achieve
d_rawmin = robot.Joint(3).VelMap.NegKnee; % Minimum raw velocity that system can achieve

%{
% Surface fit coefficients
% TODO: Put these into robot in some convenient way.
p00 =   -0.008501; % Flattening p00 to ensure that 0 joint vel = 0 sens vel
p10 =   0.000809;
p01 =   -0.2209;
p20 =   2.403e-05;
p11 =   0.0002211;
p02 =   .000196;
p30 =   8.846e-08;
p21 =   2.497e-05;
p12 =   2.542e-05;
p03 =   8.814e-06;

% Calculate d_raw (sensor velocity, volts/sec) as function of joint
% velocity (d_the) and joint angle (the).
x = the;
y = d_the;

d_raw = p00 + p10.*x + p01.*y + p20.*x.^2 + p11.*x.*y + p02.*y.^2 + p30.*x.^3 + p21.*(x.^2).*y + p12.*x.*(y.^2) + p03.*(y.^3);
%}

d_raw = mapRange(d_the, -6.328, 6.328, 1.39, -1.39);

% Clamp output
d_raw = min(max(d_raw, d_rawmin),d_rawmax);

err = [];

end

