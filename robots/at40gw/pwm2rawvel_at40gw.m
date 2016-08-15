function rawvel = pwm2rawvel_at40gw(robot, pwm)
% PWM2RAWVEL_AT40GW transforms PWM commands into raw velocities
% INPUTS:
%   pwm - Nx4 matrix, N PWM commands for the 4 joints
%
% J1 Notes (DIESEL):
%   Vel = PWM * 3.787875e-01 + -3.517091e+04
%   MaxVel: 19260 counts/sec -- not right, using 22432/sec instead
% J3 Notes (DIESEL):
%   Rotating up: + voltage, -angular, <50 pwm
%   Vel = PWM * -3.208623e-05 + 2.248113e+00
%   MaxVel: 1.39 volts/sec -- not right, using 1.75 V/s instead
% J4 Notes (DIESEL):
%   VelExtend = PWM * -2.882588e-05 + 2.069631e+00
%   VelRetract = PWM * -1.741648e-05 + 1.535560e+00
% J1 Notes (ELECTRIC):
%   Vel = PWM * 0.3499 - 31977
% TODO: should we clamp inside this function or leave it to user higher up?

rawvel = zeros(size(pwm));

% Joint 1
posi1 = pwm(:,1) > 0;
zer = pwm(:,1) == 0;
pwm(posi1,1) = mapRange(pwm(posi1,1), 0, 22432, robot.Joint(1).PWMLim.DBUpper, robot.Joint(1).PWMLim.Max);
pwm(~posi1,1) = mapRange(pwm(~posi1,1), 0, -22432, robot.Joint(1).PWMLim.DBLower, robot.Joint(1).PWMLim.Min);
pwm(zer,1) = robot.PWMZero;

% Joint 3
posi3 = pwm(:,3) > 0;
zer = pwm(:,3) == 0;
pwm(posi3,3) = mapRange(pwm(posi3,3), 0, 1.75, robot.Joint(3).PWMLim.DBLower, robot.Joint(3).PWMLim.Min);
pwm(~posi3,3) = mapRange(pwm(~posi3,3), 0, -1.75, robot.Joint(3).PWMLim.DBUpper, robot.Joint(3).PWMLim.Max);
pwm(zer,3) = robot.PWMZero;

% Joint 4
posi4 = pwm(:,4) > robot.Joint(4).PWMLim.DBUpper;
zer = pwm(:,4) < robot.Joint(4).PWMLim.DBUpper &  pwm(:,4) > robot.Joint(4).PWMLim.DBLower;
rawvel(posi4,4) = mapRange(pwm(posi4,4), robot.Joint(4).PWMLim.DBUpper, robot.Joint(4).PWMLim.Max, -0.965, 0);
rawvel(~posi4,4) = mapRange(pwm(~posi4,4), robot.Joint(4).PWMLim.DBLower, robot.Joint(4).PWMLim.Min, 0, 0.89);
rawvel(zer,4) = 0;
end
